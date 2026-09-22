#!/usr/bin/env python3
"""HTTP inference worker for the released NaVILA checkpoint, in its own venv.

Inference flow and the navigation prompt are adapted from NaVILA-Bench:
https://github.com/yang-zj1026/NaVILA-Bench/blob/
e9d2db12ce5788c0f987d734c0094100b6bc0d3a/scripts/vlm_server.py
See docs/licenses/NaVILA-Bench-MIT.txt. The HTTP transport is project code.
"""

from __future__ import annotations

import argparse
import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_factory.navila import parse_navila_action


class NaVILAInference:
    def __init__(self, model_path: str):
        import torch
        from llava.mm_utils import get_model_name_from_path
        from llava.model.builder import load_pretrained_model

        if not torch.cuda.is_available():
            raise RuntimeError("NaVILA FP16 requires an NVIDIA CUDA GPU; baseline MuJoCo does not")
        self.torch = torch
        self.tokenizer, self.model, self.processor, _ = load_pretrained_model(
            model_path=model_path,
            model_name=get_model_name_from_path(model_path),
            model_base=None,
            device_map={"": "cuda:0"},
            device="cuda",
        )
        self.model.eval()
        self.model.config.image_processor = self.processor

    def infer(self, instruction: str, frames: list[str]) -> str:
        from PIL import Image
        from llava.constants import IMAGE_TOKEN_INDEX
        from llava.conversation import SeparatorStyle, conv_templates
        from llava.mm_utils import KeywordsStoppingCriteria, process_image, tokenizer_image_token

        if not isinstance(instruction, str) or not 0 < len(instruction.strip()) <= 4000:
            raise ValueError("instruction must be a nonempty string up to 4000 characters")
        if not isinstance(frames, list) or not 1 <= len(frames) <= 8:
            raise ValueError("frames must contain 1 to 8 base64 RGB images")
        images = []
        for encoded in frames:
            if not isinstance(encoded, str):
                raise ValueError("Every frame must be a base64 string")
            with Image.open(BytesIO(base64.b64decode(encoded, validate=True))) as image:
                if image.width > 1920 or image.height > 1920:
                    raise ValueError("Image dimensions must be at most 1920 x 1920")
                images.append(image.convert("RGB"))
        # Match the official benchmark's padding for missing history.
        images = [Image.new("RGB", images[-1].size) for _ in range(8 - len(images))] + images
        processed = [process_image(image, self.model.config, None) for image in images]
        if any(image.shape != processed[0].shape for image in processed):
            raise ValueError("All processed observations must have identical shapes")
        tensor = self.torch.stack(processed).to("cuda:0", dtype=self.torch.float16)
        image_token = "<image>\n"
        query = (
            "Imagine you are a robot programmed for navigation tasks. You have been given a video "
            f'of historical observations {image_token * 7}, and current observation <image>\n. Your assigned task is: "{instruction}" '
            "Analyze this series of images to decide your next action, which could be turning left or right by a specific "
            "degree, moving forward a certain distance, or stop if the task is completed."
        )
        conversation = conv_templates["llama_3"].copy()
        conversation.append_message(conversation.roles[0], query)
        conversation.append_message(conversation.roles[1], None)
        input_ids = tokenizer_image_token(
            conversation.get_prompt(), self.tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt"
        ).unsqueeze(0).to("cuda:0")
        stop = conversation.sep2 if conversation.sep_style == SeparatorStyle.TWO else conversation.sep
        stopping = KeywordsStoppingCriteria([stop], self.tokenizer, input_ids)
        with self.torch.inference_mode():
            generated = self.model.generate(
                input_ids, images=[tensor], do_sample=False, num_beams=1,
                max_new_tokens=128, use_cache=True, stopping_criteria=[stopping],
            )
        # Some HF generation paths return prompt + continuation; VILA may return
        # continuation only. Remove a prefix only when it exactly matches.
        if generated.shape[1] >= input_ids.shape[1] and self.torch.equal(
            generated[:, : input_ids.shape[1]], input_ids
        ):
            generated = generated[:, input_ids.shape[1] :]
        return self.tokenizer.batch_decode(generated, skip_special_tokens=True)[0].strip()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=54321)
    args = parser.parse_args()
    model = NaVILAInference(str(Path(args.model_path).resolve()))

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status: int, payload: dict) -> None:
            content = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self) -> None:
            self.reply(200 if self.path == "/health" else 404, {"model": "navila-llama3-8b-8f"})

        def do_POST(self) -> None:
            if self.path != "/infer":
                self.reply(404, {"error": "Use POST /infer"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 16 * 1024 * 1024:
                    raise ValueError("Request body must be between 1 byte and 16 MiB")
                payload = json.loads(self.rfile.read(length))
                text = model.infer(payload["instruction"], payload["frames"])
                action = parse_navila_action(text)
                self.reply(200, {"text": text, "action": {"kind": action.kind, "value": action.value}})
            except (ValueError, KeyError, TypeError) as error:
                self.reply(422, {"error": str(error)})
            except Exception as error:
                self.reply(500, {"error": f"Model inference failed: {type(error).__name__}: {error}"})

    # Serialize GPU requests and bind to loopback by default.
    server = HTTPServer((args.host, args.port), Handler)
    print(f"NaVILA ready at http://{args.host}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
