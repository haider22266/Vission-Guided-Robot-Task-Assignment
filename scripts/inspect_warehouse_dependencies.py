"""Inventory authored external USD references without modifying the warehouse."""
import json
from pathlib import Path
import re
from isaacsim import SimulationApp

root=Path(__file__).resolve().parents[1]
app=SimulationApp({'headless':True})
try:
    from pxr import Sdf
    source=root/'isaac_sim/warehouse_source/warehouse_verified.usd'
    layer=Sdf.Layer.FindOrOpen(str(source))
    references=sorted(set(value for value in re.findall(r'@([^@\r\n]*)@',layer.ExportToString()) if value))
    output=root/'results/cdd/scene_asset_references.json'
    output.write_text(json.dumps({'scene':'isaac_sim/warehouse_source/warehouse_verified.usd',
        'authored_asset_paths':references,
        'note':'The scene layer is included. Referenced NVIDIA/third-party assets remain external and require access on the recipient machine.'},indent=2)+'\n')
    print(json.dumps(references,indent=2),flush=True)
finally:
    app.close()
