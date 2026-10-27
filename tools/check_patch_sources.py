"""Read-only source fingerprint / patch applicability check."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('package_root',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[1]
expected=json.loads((root/'patches/expected_sources.json').read_text());bad=[]
for rel,digest in expected.items():
 path=a.package_root/rel
 got=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
 print(('OK ' if got==digest else 'MISMATCH ')+rel)
 if got!=digest:bad.append(rel)
if bad:sys.exit('Source differs; do not apply this reviewed patch. Re-audit the actual version.')
subprocess.run(['patch','--dry-run','--batch','--fuzz=0','-p1','-d',str(a.package_root.resolve()),'-i',str(root/'patches/underlay-normal-observer.patch')],check=True)
