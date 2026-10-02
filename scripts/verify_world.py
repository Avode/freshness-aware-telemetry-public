#!/usr/bin/env python3
"""Check the scene and every locally referenced robot geometry asset."""
from pathlib import Path
import json
import os
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
scene=json.loads((ROOT/'config/scene.json').read_text())
world=ET.parse(ROOT/'worlds/plantation.sdf').getroot().find('world')
assert len(scene['robots'])==3
assert sum(x['name'].startswith('palm_') for x in scene['entities'])==64
missing=[];mesh_count=0;report=[]
for filename in [ROOT/'worlds/plantation.sdf']+[ROOT/'models'/r['model']/'model.sdf' for r in scene['robots']]:
    xml=ET.parse(filename).getroot()
    for el in xml.iter():
        if el.tag in ('uri','albedo_map','normal_map') and el.text and el.text.startswith('model://'):
            p=ROOT/'models'/el.text[len('model://'):]
            if not p.exists():missing.append(str(p))
            mesh_count+=1
    if filename.name=='model.sdf':
        model=xml.find('model')
        assert model.findtext('static')=='true',filename
        assert not model.findall('.//plugin'),filename
        assert not model.findall('.//sensor'),filename
        report.append(dict(model=model.get('name'),links=len(model.findall('link')),joints=len(model.findall('joint'))))
assert not missing,missing
env=dict(os.environ,SDF_PATH=str(ROOT/'models'),IGN_GAZEBO_RESOURCE_PATH=str(ROOT/'models'))
subprocess.run(['ign','sdf','-k',str(ROOT/'worlds/plantation.sdf')],env=env,check=True)
result=dict(estate_models=len(scene['entities']),palms=scene['palm_count'],robots=report,
            checked_asset_references=mesh_count,missing_assets=missing,controllers=False,telemetry=False)
(ROOT/'docs/validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
