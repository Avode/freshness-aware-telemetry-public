#!/usr/bin/env python3
"""Recreate the pinned upstream model cache without installing robot drivers."""
import json
import os
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
UP=Path(os.environ.get('FLEETSCOPE_ASSETS', ROOT/'assets')).expanduser()/'upstream'
SPARSE={
    'unitree_ros':['robots/go2_description'],
    'px4_models':['models/x500','models/x500_base'],
    'kortex':['kortex_description'],
    'clearpath_common':['clearpath_platform_description','clearpath_mounts_description'],
}
UP.mkdir(parents=True,exist_ok=True)
for name,spec in json.loads((ROOT/'config/robot_sources.json').read_text()).items():
    dest=UP/name
    def git(*args):
        return subprocess.run(['git','-C',str(dest),*args],check=True)
    if not (dest/'.git').exists():
        dest.mkdir(exist_ok=True)
        git('init')
        git('remote','add','origin',spec['url'])
        git('sparse-checkout','init','--cone')
        git('sparse-checkout','set',*SPARSE[name])
        git('fetch','--depth','1','--filter=blob:none','origin',spec['commit'])
        git('checkout','--detach','FETCH_HEAD')
    actual=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip()
    if actual!=spec['commit']:
        raise SystemExit(f'{dest} has a different revision; leaving it untouched.')
    print(name,actual)
