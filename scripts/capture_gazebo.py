#!/usr/bin/env python3
"""Capture only the Gazebo window for visual verification on this X11 desktop."""
import argparse
import re
import subprocess
from pathlib import Path
import gi
gi.require_version('Gdk','3.0')
gi.require_version('GdkX11','3.0')
from gi.repository import Gdk, GdkX11

parser=argparse.ArgumentParser()
parser.add_argument('output',type=Path)
args=parser.parse_args()
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
match=re.search(r'(0x[0-9a-f]+) "Gazebo":',tree)
if not match:raise SystemExit('No Gazebo window is open.')
window=GdkX11.X11Window.foreign_new_for_display(Gdk.Display.get_default(),int(match[1],16))
pixbuf=Gdk.pixbuf_get_from_window(window,0,0,window.get_width(),window.get_height())
args.output.parent.mkdir(parents=True,exist_ok=True)
pixbuf.savev(str(args.output),'png',[],[])
