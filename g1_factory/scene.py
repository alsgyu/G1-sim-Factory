"""Editable semantic warehouse + office for the upstream Unitree G1.

All props are procedural original MJCF geometry. Racks use conservative box
colliders and kinematic mocap bodies. Temperatures are synthetic task signals.
Coordinates in configuration and metadata are world coordinates, in metres.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import re
from typing import Any
import xml.etree.ElementTree as ET

import yaml


def _number(value: Any, label: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    value = float(value)
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f"{label} must be finite and >= {minimum}")
    return value


def _vector(value: Any, n: int, label: str) -> list[float]:
    if not isinstance(value, (tuple, list)) or len(value) != n:
        raise ValueError(f"{label} must contain {n} numbers")
    return [_number(v, label) for v in value]


def _identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value):
        raise ValueError(f"{label} must start with a letter and contain only letters, digits or _")
    return value


def _shelf_half_extent(size, yaw):
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return ((c * size[0] + s * size[1]) / 2, (s * size[0] + c * size[1]) / 2)


def _segment_hits_shelf(a, b, shelf, clearance):
    yaw = shelf.get("yaw", 0.0)
    c, s = math.cos(yaw), math.sin(yaw)
    def local(p):
        x, y = p[0] - shelf["position"][0], p[1] - shelf["position"][1]
        return c*x+s*y, -s*x+c*y
    p, q = local(a), local(b)
    t0, t1 = 0.0, 1.0
    for axis in (0, 1):
        bound = shelf["size"][axis]/2 + clearance
        delta = q[axis]-p[axis]
        if abs(delta) < 1e-12:
            if abs(p[axis]) > bound:
                return False
        else:
            lo, hi = sorted(((-bound-p[axis])/delta, (bound-p[axis])/delta))
            t0, t1 = max(t0, lo), min(t1, hi)
            if t0 > t1:
                return False
    return True


def _inside(bounds, position, extent=(0, 0), margin=0.0):
    return all(bounds[i]+extent[i]+margin <= position[i] <= bounds[i+2]-extent[i]-margin for i in (0, 1))


def load_config(config_path: str | Path) -> dict:
    with Path(config_path).expanduser().open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    validate_config(config)
    return config


def validate_config(config: dict) -> None:
    if not isinstance(config, dict):
        raise ValueError("Factory configuration must be a mapping")
    layout = config.get("layout", {})
    for name in ("warehouse_bounds", "office_bounds"):
        b = _vector(layout.get(name), 4, "layout."+name)
        if b[2]-b[0] < 12 or b[3]-b[1] < 12:
            raise ValueError(f"layout.{name} dimensions must be at least 12 metres")
    for key in ("corridor_width", "wall_thickness", "wall_height", "robot_clearance"):
        _number(layout.get(key), "layout."+key, .01)
    if layout["corridor_width"] <= 2*layout["robot_clearance"]:
        raise ValueError("corridor_width must exceed twice robot_clearance")
    # Structural geometry currently follows this benchmark footprint. Reject
    # silent partial rescaling; props and paths remain freely editable.
    if list(layout["warehouse_bounds"]) != [-24,-16,24,16] or list(layout["office_bounds"]) != [28,-12,52,12]:
        raise ValueError("This layout uses warehouse [-24,-16,24,16] and office [28,-12,52,12]; edit geometry to resize")
    if any(not math.isclose(layout[key], expected) for key, expected in
           (("corridor_width", 4.0), ("wall_thickness", .2), ("wall_height", 6.0))):
        raise ValueError("Structural benchmark dimensions are fixed: corridor_width=4, wall_thickness=0.2, wall_height=6; edit geometry to resize")
    sim = config.get("simulation", {})
    if _number(sim.get("timestep", .002), "simulation.timestep", .00001) > .01:
        raise ValueError("Use a simulation timestep <= 0.01 s for this G1 model")
    friction = _vector(sim.get("floor_friction", [1,.005,.0001]),3,"simulation.floor_friction")
    if min(friction)<0 or friction[0]==0:
        raise ValueError("Floor sliding friction must be positive; other friction >= 0")
    temp = config.get("temperature", {})
    _number(temp.get("ambient_c",23),"temperature.ambient_c")
    _number(temp.get("temporal_amplitude_c",0),"temperature.temporal_amplitude_c",0)
    _number(temp.get("period_s",120),"temperature.period_s",.01)
    zones = config.get("zones", [])
    if not isinstance(zones,list) or len(zones)!=8:
        raise ValueError("Exactly four warehouse zones and four office rooms are required")
    seen=set()
    for zone in zones:
        zid=_identifier(zone.get("id"),"zone.id")
        if zid in seen:
            raise ValueError(f"Duplicate zone id: {zid}")
        seen.add(zid)
        if zone.get("kind") not in ("warehouse","office"):
            raise ValueError(f"{zid}.kind must be warehouse or office")
        bounds=_vector(zone.get("bounds"),4,f"{zid}.bounds")
        if bounds[0]>=bounds[2] or bounds[1]>=bounds[3]:
            raise ValueError(f"{zid}.bounds must be xmin,ymin,xmax,ymax")
        for key in ("center","goal"):
            point=_vector(zone.get(key),2,f"{zid}.{key}")
            if not _inside(bounds,point,margin=layout["robot_clearance"] if key=="goal" else 0):
                raise ValueError(f"{zid}.{key} must be inside its zone")
        shelf_ids=set()
        for shelf in zone.get("shelves",[]):
            sid=_identifier(shelf.get("id"),f"{zid}.shelf.id")
            if sid in shelf_ids:
                raise ValueError(f"Duplicate shelf id: {zid}.{sid}")
            shelf_ids.add(sid)
            p=_vector(shelf.get("position"),2,f"{zid}.{sid}.position")
            size=_vector(shelf.get("size"),3,f"{zid}.{sid}.size")
            yaw=_number(shelf.get("yaw",0),"shelf.yaw")
            if min(size)<.12 or size[2]>layout["wall_height"]:
                raise ValueError(f"{zid}.{sid} size must be >= 0.12 m and below wall height")
            if not _inside(bounds,p,_shelf_half_extent(size,yaw),layout["wall_thickness"]/2):
                raise ValueError(f"{zid}.{sid} extends beyond zone boundary")
            if _segment_hits_shelf(zone["goal"],[zone["goal"][0],0],shelf,layout["robot_clearance"]):
                raise ValueError(f"{zid}.{sid} blocks the zone's default approach route")
        sensor_ids=set()
        for sensor in zone.get("sensors",[]):
            sid=_identifier(sensor.get("id"),f"{zid}.sensor.id")
            if sid in sensor_ids:
                raise ValueError(f"Duplicate sensor id: {zid}.{sid}")
            sensor_ids.add(sid)
            p=_vector(sensor.get("position"),3,f"{zid}.{sid}.position")
            if not _inside(bounds,p) or not 0<p[2]<=layout["wall_height"]:
                raise ValueError(f"{zid}.{sid} sensor must be inside its zone and above ground")
        for hotspot in zone.get("hotspots",[]):
            p=_vector(hotspot.get("position"),2,"hotspot.position")
            if not _inside(bounds,p):
                raise ValueError("hotspot must be inside its zone")
            _number(hotspot.get("sigma_m"),"hotspot.sigma_m",.01)
            _number(hotspot.get("amplitude_c"),"hotspot.amplitude_c")
    for sid,sequence in config.get("scenarios",{}).items():
        _identifier(sid,"scenario.id")
        if not isinstance(sequence,list) or not sequence or any(z not in seen for z in sequence):
            raise ValueError(f"{sid} must contain valid zone ids")


def _fmt(values):
    return " ".join(f"{float(v):.9g}" for v in values)


def _box(parent,name,position,halfsize,rgba,**extra):
    attrs=dict(name=name,type="box",pos=_fmt(position),size=_fmt(halfsize),rgba=_fmt(rgba),contype="1",conaffinity="1")
    attrs.update({k:str(v) for k,v in extra.items()})
    return ET.SubElement(parent,"geom",**attrs)


def _vbox(parent,name,position,halfsize,rgba,**extra):
    return _box(parent,name,position,halfsize,rgba,contype="0",conaffinity="0",**extra)


def _obstacle(metadata,name,x,y,hx,hy):
    metadata["obstacles"].append({"id":name,"bounds":[x-hx,y-hy,x+hx,y+hy]})


def _solid(world,metadata,name,position,halfsize,rgba,**extra):
    _box(world,name,position,halfsize,rgba,**extra)
    if position[2]-halfsize[2]<1.8 and position[2]+halfsize[2]>.08:
        _obstacle(metadata,name,*position[:2],*halfsize[:2])


def _absolute_assets(root,robot_file):
    compiler=root.find("compiler")
    if compiler is None:
        compiler=ET.SubElement(root,"compiler",angle="radian")
    assetdir=compiler.get("assetdir","")
    for tag,directory in (("mesh","meshdir"),("texture","texturedir")):
        asset_dir=(robot_file.parent/compiler.get(directory,assetdir)).resolve()
        for asset in root.findall(f".//asset/{tag}"):
            filename=asset.get("file")
            if filename:
                resolved=(asset_dir/filename).resolve()
                if not resolved.is_file():
                    raise FileNotFoundError(f"Missing upstream {tag}: {resolved}")
                asset.set("file",str(resolved))
        compiler.attrib.pop(directory,None)
    compiler.attrib.pop("assetdir",None)
    compiler.set("strippath","false")


def _camera(world,name,position,target,fovy=50):
    import numpy as np
    z=np.asarray(position,dtype=float)-np.asarray(target,dtype=float)
    z/=np.linalg.norm(z)
    x=np.cross([0,0,1],z)
    if np.linalg.norm(x)<1e-8:
        x=np.asarray([1.,0,0])
    else:
        x/=np.linalg.norm(x)
    y=np.cross(z,x)
    ET.SubElement(world,"camera",name=name,pos=_fmt(position),xyaxes=_fmt([*x,*y]),fovy=str(fovy))


def _sign(asset,world,output_dir,name,text,position,width,height,color=(25,40,49),upright=False):
    """Readable original sign artwork, generated locally with Pillow."""
    from PIL import Image,ImageDraw,ImageFont
    folder=Path(output_dir)/"scene_assets"
    folder.mkdir(parents=True,exist_ok=True)
    path=folder/(name+".png")
    img=Image.new("RGB",(1024,192),color)
    draw=ImageDraw.Draw(img)
    font_paths=("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf","/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf")
    font=next((ImageFont.truetype(p,74) for p in font_paths if Path(p).exists()),ImageFont.load_default())
    box=draw.textbbox((0,0),text,font=font)
    if box[2]-box[0]>960:
        font=ImageFont.truetype(font.path,int(74*960/(box[2]-box[0]))) if hasattr(font,"path") else font
    draw.rectangle((8,8,1015,183),outline=(235,194,76),width=5)
    draw.text((512,96),text,anchor="mm",fill=(250,250,244),font=font)
    img.save(path)
    ET.SubElement(asset,"texture",name=name+"_texture",type="2d",file=str(path.resolve()))
    ET.SubElement(asset,"material",name=name+"_material",texture=name+"_texture",texuniform="false",reflectance="0",specular="0",shininess="0")
    extra={"material":name+"_material"}
    if upright:
        extra["euler"]="1.57079632679 0 0"
    _vbox(world,name,position,[width/2,height/2,.006],(1,1,1,1),**extra)


def _rack(world,metadata,zid,shelf):
    name=f"factory_shelf_{zid}_{shelf['id']}"
    x,y=shelf["position"]; sx,sy,sz=shelf["size"]; yaw=shelf.get("yaw",0)
    body=ET.SubElement(world,"body",name=name,mocap="true",pos=_fmt([x,y,0]),quat=_fmt([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
    _box(body,name+"_collision",[0,0,sz/2],[sx/2,sy/2,sz/2],(0,0,0,0),group="3")
    for ix,px in enumerate((-sx/2+.06,sx/2-.06)):
        for iy,py in enumerate((-sy/2+.05,sy/2-.05)):
            _vbox(body,f"{name}_post_{ix}_{iy}",[px,py,sz/2],[.055,.055,sz/2],(.10,.22,.29,1))
    for level in range(4):
        z=.16+level*(sz-.65)/4
        _vbox(body,f"{name}_board_{level}",[0,0,z],[sx/2,sy/2,.045],(.44,.48,.48,1))
        for side in (-1,1):
            _vbox(body,f"{name}_beam_{level}_{side}",[0,side*(sy/2-.04),z-.08],[sx/2,.045,.085],(.90,.43,.13,1))
        for index,px in enumerate((-sx*.29,0,sx*.29)):
            _vbox(body,f"{name}_pallet_{level}_{index}",[px,0,z+.085],[sx*.135,sy*.43,.04],(.48,.36,.22,1))
            h=.30+.065*((level+index)%3)
            _vbox(body,f"{name}_stock_{level}_{index}",[px,0,z+.13+h],[sx*.126,sy*.40,h],(.62+.035*index,.48+.025*level,.31+.018*index,1))
            _vbox(body,f"{name}_strap_{level}_{index}",[px,0,z+.135+2*h],[.045,sy*.405,.005],(.85,.78,.56,1))
    hx,hy=_shelf_half_extent(shelf["size"],yaw)
    _obstacle(metadata,name,x,y,hx,hy)
    metadata["zones"][zid]["shelves"][shelf["id"]]={"body_name":name,**copy.deepcopy(shelf)}


def _workbench(world,metadata,name,x,y,width=3.4,depth=1.4,inspection=False):
    _solid(world,metadata,name+"_collision",[x,y,.59],[width/2,depth/2,.59],(0,0,0,0),group="3")
    _vbox(world,name+"_top",[x,y,1.13],[width/2,depth/2,.065],(.78,.80,.79,1))
    for i,dx in enumerate((-width/2+.1,width/2-.1)):
        for j,dy in enumerate((-depth/2+.1,depth/2-.1)):
            _vbox(world,f"{name}_leg_{i}_{j}",[x+dx,y+dy,.53],[.075,.075,.53],(.18,.26,.31,1))
    if inspection:
        _vbox(world,name+"_gantry_a",[x-.8,y,1.7],[.065,.08,.5],(.28,.35,.38,1))
        _vbox(world,name+"_gantry_b",[x+.8,y,1.7],[.065,.08,.5],(.28,.35,.38,1))
        _vbox(world,name+"_gantry_top",[x,y,2.22],[.87,.08,.065],(.28,.35,.38,1))
        _vbox(world,name+"_camera",[x,y,2.06],[.13,.13,.15],(.11,.13,.15,1))
        _vbox(world,name+"_part",[x,y,1.3],[.45,.35,.1],(.41,.52,.59,1))
    else:
        _vbox(world,name+"_toolboard",[x,y+depth/2-.04,1.63],[width/2,.06,.4],(.24,.36,.41,1))
        for i in range(5):
            _vbox(world,f"{name}_tool_{i}",[x-width*.32+i*width*.16,y+depth/2-.12,1.63],[.035,.03,.22],(.77,.73,.56,1))
        _vbox(world,name+"_part",[x,y,1.30],[.55,.35,.12],(.44,.49,.50,1))


def _conveyor(world,metadata,name,x,y,length=9):
    _solid(world,metadata,name+"_collision",[x,y,.56],[length/2,.78,.56],(0,0,0,0),group="3")
    _vbox(world,name+"_belt",[x,y,1.05],[length/2,.7,.1],(.12,.18,.21,1))
    for side in (-1,1):
        _vbox(world,f"{name}_rail_{side}",[x,y+side*.77,1.05],[length/2,.06,.15],(.61,.64,.63,1))
    for index in range(4):
        dx=-length*.38+index*length*.25
        for side in (-1,1):
            _vbox(world,f"{name}_leg_{index}_{side}",[x+dx,y+side*.55,.5],[.075,.075,.5],(.33,.40,.43,1))
        _vbox(world,f"{name}_package_{index}",[x+dx,y,1.39],[.55,.45,.24],(.72,.55,.34,1))
        _vbox(world,f"{name}_tape_{index}",[x+dx,y,1.635],[.07,.45,.005],(.90,.79,.53,1))


def _cart(world,metadata,name,x,y):
    _solid(world,metadata,name+"_base",[x,y,.35],[.8,.55,.22],(.26,.35,.37,1))
    _vbox(world,name+"_deck",[x,y,.59],[.76,.52,.04],(.79,.79,.68,1))
    for ix,dx in enumerate((-.6,.6)):
        for iy,dy in enumerate((-.50,.50)):
            ET.SubElement(world,"geom",name=f"{name}_wheel_{ix}_{iy}",type="cylinder",pos=_fmt([x+dx,y+dy,.2]),size=".19 .07",euler="1.57079632679 0 0",rgba=".10 .12 .13 1",contype="0",conaffinity="0")
    _vbox(world,name+"_bumper",[x-.83,y,.35],[.055,.57,.1],(.87,.64,.18,1))
    _vbox(world,name+"_lidar",[x+.5,y,.72],[.12,.12,.1],(.15,.18,.20,1))


def _warehouse(world,asset,config,metadata,output_dir):
    metal=(.26,.33,.37,1); wall=(.70,.74,.75,1); yellow=(.97,.72,.21,1)
    # Separate paint geometry never catches G1's feet.
    _vbox(world,"factory_warehouse_slab",[0,0,-.022],[24,16,.02],(.52,.56,.57,1))
    for zid,color in (("material_storage",(.61,.60,.55,1)),("assembly_line",(.49,.55,.57,1)),("inspection",(.56,.60,.56,1)),("charging",(.62,.59,.50,1))):
        zone=metadata["zones"][zid]; b=zone["bounds"]
        _vbox(world,f"factory_{zid}_floor",[(b[0]+b[2])/2,(b[1]+b[3])/2,.002],[(b[2]-b[0])/2,(b[3]-b[1])/2,.002],color)
        for y in (b[1]+.2,b[3]-.2):
            _vbox(world,f"factory_{zid}_stripe_{y}",[(b[0]+b[2])/2,y,.007],[(b[2]-b[0])/2-.15,.045,.002],yellow)
    _vbox(world,"factory_main_cross_aisle_x",[2,0,.008],[26,1.85,.002],(.33,.38,.39,1))
    _vbox(world,"factory_main_cross_aisle_y",[0,0,.009],[1.85,16,.002],(.33,.38,.39,1))
    for y in (-1.91,1.91):
        _vbox(world,f"factory_aisle_edge_y{y}",[2,y,.012],[26,.045,.002],yellow)
    for x in (-1.91,1.91):
        _vbox(world,f"factory_aisle_edge_x{x}",[x,0,.012],[.045,16,.002],yellow)
    for index,x in enumerate(range(-22,28,3)):
        _vbox(world,f"factory_lane_dash_{index}",[x,0,.013],[.6,.035,.002],(.88,.89,.84,1))
    # Tall north/west shell; south/east use cutaway parapets for overview.
    _solid(world,metadata,"factory_north_wall",[0,16,3],[24,.10,3],wall)
    _solid(world,metadata,"factory_west_wall",[-24,0,3],[.10,16,3],wall)
    _solid(world,metadata,"factory_south_cutaway",[0,-16,.65],[24,.1,.65],wall)
    for y in (-9,9):
        _solid(world,metadata,f"factory_east_cutaway_{y}",[24,y,.65],[.1,7,.65],wall)
    # A physically continuous bridge into the office; no steps or thresholds.
    for y in (-2.1,2.1):
        _solid(world,metadata,f"factory_bridge_rail_{y}",[26,y,.45],[2,.08,.45],metal)
    # Structural columns remain outside the central and zone entrance aisles.
    for index,(x,y) in enumerate([(x,y) for x in (-23,0,23) for y in (-15,15)]):
        _solid(world,metadata,f"factory_column_{index}",[x,y,3],[.15,.15,3],metal)
        _vbox(world,f"factory_column_foot_{index}",[x,y,.38],[.23,.23,.38],yellow)
    for index,y in enumerate((-14.5,0,14.5)):
        _vbox(world,f"factory_truss_{index}",[0,y,6.15],[23.8,.075,.09],metal)
        for j,x in enumerate((-16,-8,0,8,16)):
            _vbox(world,f"factory_luminaire_{index}_{j}",[x,y,5.85],[1.15,.14,.04],(.91,.93,.85,1))
    # North loading doors and ribbed panels visually distinguish an industrial shell.
    for index,x in enumerate((-19,-11,7,17)):
        _vbox(world,f"factory_loading_frame_{index}",[x,15.83,2.5],[2.0,.08,2.5],metal)
        _vbox(world,f"factory_loading_door_{index}",[x,15.72,2.35],[1.8,.04,2.2],(.50,.56,.58,1))
        for j in range(12):
            _vbox(world,f"factory_loading_rib_{index}_{j}",[x,15.665,.35+j*.35],[1.76,.017,.025],(.37,.43,.46,1))
        _sign(asset,world,output_dir,f"factory_loading_sign_{index}",f"DOCK {index+1:02d}",[x,15.54,5.25],3,.56,upright=True)
    labels={"material_storage":"01  MATERIAL STORAGE","assembly_line":"02  ASSEMBLY LINE","inspection":"03  QUALITY INSPECTION","charging":"04  CHARGING STATION"}
    for zid,label in labels.items():
        zone=metadata["zones"][zid]
        gx,gy=zone["goal"]
        _sign(asset,world,output_dir,f"factory_sign_{zid}",label,[gx,5.7 if gy>0 else -5.7,.021],13,1.5)
        for shelf in next(z for z in config["zones"] if z["id"]==zid).get("shelves",[]):
            _rack(world,metadata,zid,shelf)
    # Assembly island: two production conveyors, tool benches and machine cells.
    for index,y in enumerate((8.2,12.1)):
        _conveyor(world,metadata,f"factory_assembly_conveyor_{index}",10,y,10)
    for index,y in enumerate((8.1,12.0)):
        _workbench(world,metadata,f"factory_assembly_bench_{index}",20,y,3.2)
        _solid(world,metadata,f"factory_assembly_machine_{index}",[3.8,y,1.45],[.8,1.15,1.45],(.38,.48,.51,1))
        _vbox(world,f"factory_assembly_machine_window_{index}",[3.8,y-1.16,1.85],[.58,.015,.55],(.13,.23,.28,1))
        _vbox(world,f"factory_assembly_machine_control_{index}",[4.46,y-1.19,1.25],[.09,.055,.21],(.79,.73,.45,1))
    # Inspection: vision gantries, metrology cabinets and pass/fail bins.
    for index,(x,y) in enumerate(((-18,-8.2),(-10,-8.2),(-18,-12.4),(-10,-12.4))):
        _workbench(world,metadata,f"factory_inspection_station_{index}",x,y,4,1.8,True)
        _solid(world,metadata,f"factory_inspection_bin_{index}",[x+2.6,y,.55],[.48,.55,.55],(.28,.42,.42,1))
    for index,y in enumerate((-8,-12)):
        _solid(world,metadata,f"factory_inspection_cabinet_{index}",[-4,y,1.05],[.65,1.1,1.05],(.70,.73,.71,1))
        _vbox(world,f"factory_inspection_screen_{index}",[-4,y+1.12,1.5],[.46,.025,.3],(.09,.24,.28,1))
    # Charging: clearly marked parking bays and parked AMR carts.
    for index,x in enumerate((5,9,13,17,21)):
        _solid(world,metadata,f"factory_charger_{index}",[x,-13.4,1.05],[.48,.30,1.05],(.25,.34,.37,1))
        _vbox(world,f"factory_charger_screen_{index}",[x,-13.085,1.45],[.31,.018,.22],(.16,.47,.45,1))
        _vbox(world,f"factory_charger_light_{index}",[x,-13.075,1.83],[.18,.025,.025],(.48,.78,.59,1))
        for side in (-1,1):
            _vbox(world,f"factory_parking_{index}_{side}",[x+side*1.25,-10.9,.02],[.045,2,.002],yellow)
        _vbox(world,f"factory_parking_back_{index}",[x,-12.9,.02],[1.25,.045,.002],yellow)
        if index!=2:
            _cart(world,metadata,f"factory_amr_{index}",x,-10.4)
    for index,x in enumerate((7,15)):
        _solid(world,metadata,f"factory_battery_cabinet_{index}",[x,-7.1,.8],[1.0,.45,.8],(.65,.68,.61,1))
        _vbox(world,f"factory_battery_cabinet_label_{index}",[x,-6.635,.95],[.65,.018,.20],(.88,.68,.23,1))
    # Pallet staging along the far storage wall, away from the delivery route.
    for index,x in enumerate((-21.0,-16.0,-6.0)):
        _solid(world,metadata,f"factory_staging_pallet_{index}",[x,4.15,.4],[.85,.65,.4],(.62,.48,.29,1))
        _vbox(world,f"factory_staging_strap_{index}",[x,4.15,.81],[.055,.65,.01],(.85,.79,.60,1))
    _camera(world,"warehouse_overview",[41,-44,39],[0,0,1.8],51)
    _camera(world,"warehouse_aisle",[21,-1.0,2.1],[-16,5,2.3],66)
    metadata["camera_names"].update(warehouse="warehouse_overview",warehouse_aisle="warehouse_aisle")


def build_scene(config_path: str | Path, upstream_root: str | Path, output_path: str | Path) -> dict:
    """Write MJCF and return global semantic zones, assets and navigation metadata."""
    config=load_config(config_path)
    robot_file=Path(upstream_root).expanduser().resolve()/"resources/robots/g1_description/g1_12dof.xml"
    if not robot_file.is_file():
        raise FileNotFoundError(f"G1 model not found: {robot_file}; run python scripts/fetch_assets.py")
    root=ET.parse(robot_file).getroot(); root.set("model","G1_semantic_warehouse_office")
    _absolute_assets(root,robot_file)
    option=root.find("option")
    if option is None: option=ET.SubElement(root,"option")
    option.set("timestep",str(config.get("simulation",{}).get("timestep",.002))); option.set("gravity","0 0 -9.81")
    world=root.find("worldbody")
    if world is None: raise ValueError("Upstream G1 XML has no worldbody")
    pelvis=world.find(".//body[@name='pelvis']")
    if pelvis is None: raise ValueError("Upstream G1 XML has no pelvis body")
    ET.SubElement(pelvis,"camera",name="ego_rgb",pos="0.15 0 0.50",xyaxes="0 -1 0 0 0 1",fovy="75")
    for geom in list(world.findall("geom")):
        if geom.get("type")=="plane": world.remove(geom)
    asset=root.find("asset")
    if asset is None: asset=ET.SubElement(root,"asset")
    ET.SubElement(asset,"texture",name="factory_sky",type="skybox",builtin="gradient",
                  rgb1=".58 .66 .70",rgb2=".88 .91 .91",width="512",height="3072")
    output=Path(output_path).expanduser().resolve(); output.parent.mkdir(parents=True,exist_ok=True)
    metadata={"scene_path":str(output),"config":copy.deepcopy(config),"zones":{},"obstacles":[],"camera_names":{"ego":"ego_rgb","map":"map_camera"},"sensor_names":[],"synthetic_temperature":True,"bounds":[-25,-17,53,17],"warehouse_bounds":[-24,-16,24,16],"office_bounds":[28,-12,52,12]}
    for z in config["zones"]:
        metadata["zones"][z["id"]]={**copy.deepcopy(z),"shelves":{},"sensors":[],"origin":[0,0]}
    ET.SubElement(world,"geom",name="factory_floor",type="plane",pos="14 0 0",size="40 18 .1",rgba=".66 .69 .69 1",contype="1",conaffinity="1",condim="3",friction=_fmt(config.get("simulation",{}).get("floor_friction",[1,.005,.0001])))
    for index,(x,y) in enumerate(((-12,8),(12,-8),(40,0))):
        ET.SubElement(world,"light",name=f"factory_light_{index}",pos=_fmt([x,y,14]),dir="0 0 -1",directional="true",diffuse=".18 .18 .17",ambient=".025 .025 .025",castshadow="true" if index==0 else "false")
    _camera(world,"map_camera",[14,0,94],[14,0,0],51)
    _camera(world,"site_overview",[74,-67,65],[12,0,1.3],53)
    metadata["camera_names"]["site"]="site_overview"
    visual=root.find("visual")
    if visual is None: visual=ET.SubElement(root,"visual")
    gl=visual.find("global")
    if gl is None: gl=ET.SubElement(visual,"global")
    gl.set("offwidth","1600"); gl.set("offheight","1200")
    head=visual.find("headlight")
    if head is None: head=ET.SubElement(visual,"headlight")
    head.set("ambient",".15 .15 .15"); head.set("diffuse",".28 .28 .28")
    # Unitree's model retains robot-sized statistics (extent ~1.23 m). Its
    # default 50*extent far plane clips a campus overview. Keep a near plane
    # appropriate for ego RGB, and explicitly extend the distant clip plane.
    clip=visual.find("map")
    if clip is None: clip=ET.SubElement(visual,"map")
    clip.set("znear",".005"); clip.set("zfar","500")
    _warehouse(world,asset,config,metadata,output.parent)
    from g1_factory.office import add_office
    add_office(world,asset,config,metadata,output.parent)
    for zone in config["zones"]:
        zid=zone["id"]
        for sensor in zone.get("sensors",[]):
            name=f"factory_sensor_{zid}_{sensor['id']}"; position=sensor["position"]
            ET.SubElement(world,"site",name=name,type="box",pos=_fmt(position),size=".07 .035 .11",rgba=".85 .32 .15 1")
            _vbox(world,name+"_housing",position,[.09,.04,.13],(.91,.89,.77,1))
            metadata["sensor_names"].append(name)
            metadata["zones"][zid]["sensors"].append({"id":sensor["id"],"site_name":name,"position":list(position),"local_position":list(position)})
    # Compatibility alias refers to semantic zones, never laboratory room copies.
    metadata["bays"]=metadata["zones"]
    from g1_factory.routes import build_scenarios
    metadata["scenarios"]=build_scenarios(metadata)
    ET.indent(root,space="  ")
    ET.ElementTree(root).write(output,encoding="utf-8",xml_declaration=True)
    return metadata


def move_shelf(model,data,metadata: dict,zone_id: str,shelf_id: str,world_xy,yaw: float=0.0) -> None:
    """Move rack to WORLD x/y, while paused.

    This authoring action does not claim G1 manipulation. The caller must replan
    after edits. Boundary, other obstacle and G1 intersections are rejected.
    """
    import mujoco
    position=_vector(world_xy,2,"shelf.position"); yaw=_number(yaw,"shelf.yaw")
    zone=metadata["zones"][zone_id]; shelf=zone["shelves"][shelf_id]
    extent=_shelf_half_extent(shelf["size"],yaw)
    if not _inside(zone["bounds"],position,extent,.1):
        raise ValueError("Shelf extends beyond zone boundary")
    for other in metadata["obstacles"]:
        if other["id"]==shelf["body_name"]: continue
        b=other["bounds"]
        if position[0]+extent[0]+.05>b[0] and position[0]-extent[0]-.05<b[2] and position[1]+extent[1]+.05>b[1] and position[1]-extent[1]-.05<b[3]:
            raise ValueError(f"Shelf placement overlaps {other['id']}")
    robot_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"pelvis")
    if robot_id>=0 and _segment_hits_shelf(list(data.xpos[robot_id,:2]),list(data.xpos[robot_id,:2]),{"position":position,"size":shelf["size"],"yaw":yaw},metadata["config"]["layout"]["robot_clearance"]):
        raise ValueError("Shelf placement overlaps G1; move the robot first")
    body_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,shelf["body_name"])
    if body_id<0 or model.body_mocapid[body_id]<0: raise ValueError(f"Missing mocap shelf body {shelf['body_name']}")
    mocap_id=model.body_mocapid[body_id]; data.mocap_pos[mocap_id]=[*position,0]
    data.mocap_quat[mocap_id]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
    shelf["position"],shelf["yaw"]=position,yaw
    for other in metadata["obstacles"]:
        if other["id"]==shelf["body_name"]:
            other["bounds"]=[position[0]-extent[0],position[1]-extent[1],position[0]+extent[0],position[1]+extent[1]]
    mujoco.mj_forward(model,data)


def temperature_readings(metadata: dict,time_s: float=0.0) -> list[dict]:
    """Deterministic synthetic deg C values; not thermodynamic simulation."""
    time_s=_number(time_s,"time_s",0); cfg=metadata["config"]; temp=cfg.get("temperature",{})
    ambient=temp.get("ambient_c",23); fluctuation=temp.get("temporal_amplitude_c",0)*math.sin(2*math.pi*time_s/temp.get("period_s",120))
    readings=[]
    for zone in cfg["zones"]:
        zid=zone["id"]
        for sensor in metadata["zones"][zid]["sensors"]:
            x,y=sensor["position"][:2]; value=ambient+fluctuation
            for hot in zone.get("hotspots",[]):
                distance2=(x-hot["position"][0])**2+(y-hot["position"][1])**2
                value+=hot["amplitude_c"]*math.exp(-distance2/(2*hot["sigma_m"]**2))
            readings.append({"sensor_id":sensor["site_name"],"zone_id":zid,"bay_id":zid,"label":zone["label"],"position":sensor["position"],"time_s":time_s,"temperature_c":float(value),"synthetic":True})
    return readings
