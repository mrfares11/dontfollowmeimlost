from pathlib import Path
import ast
import re

inp = Path("smol.txt")
out = Path("smol.sdf")

text = inp.read_text(encoding="utf-8")

m = re.search(r'data:\s*"(.*)"\s*$', text, re.DOTALL)
if not m:
    raise RuntimeError("Could not find data: \"...\" in generated_world_output.txt")

escaped = '"' + m.group(1) + '"'
sdf_text = ast.literal_eval(escaped)

out.write_text(sdf_text, encoding="utf-8")
print(f"Saved cleaned SDF to {out}")
