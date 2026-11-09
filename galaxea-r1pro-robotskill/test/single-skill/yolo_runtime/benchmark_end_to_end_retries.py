import pathlib,subprocess,json,time
r=pathlib.Path(__file__).resolve().parent
records=[json.loads((r/'end-to-end-20260928.json').read_text())]
for _ in range(3):
 subprocess.run(['python3',str(r/'benchmark_end_to_end.py')],stdout=subprocess.DEVNULL,check=True,timeout=185)
 d=json.loads((r/'end-to-end-20260928.json').read_text());records.append(d);print(json.dumps(d),flush=True)
 if d.get('overall_status')=='succeeded':break
(r/'end-to-end-attempts-20260928.json').write_text(json.dumps(records,indent=2))
