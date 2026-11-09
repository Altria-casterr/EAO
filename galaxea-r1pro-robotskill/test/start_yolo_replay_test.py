"""Start the prepared replay-only test framework on localhost:8082."""
import json,os,pathlib,subprocess,time,urllib.request,urllib.error,uuid

def main():
    root=pathlib.Path(__file__).resolve().parent;runtime=root/'single-skill/yolo_runtime'
    cwd=runtime/'replay-framework';cwd.mkdir(exist_ok=True)
    config=cwd/'config.yaml'
    config.write_text('framework_name: yolo-replay-test\nhttp_ip: 127.0.0.1\nhttp_port: 8082\n')
    base='http://127.0.0.1:8082'
    def get(path):
        with urllib.request.urlopen(base+path,timeout=3) as r:return json.load(r)
    try:existing=get('/api/config')
    except urllib.error.URLError:
        workspace=root.parents[2]
        binary=os.environ.get('ABILITY_FRAMEWORK_BINARY',str(workspace/'semantic-ability/ability-runtime/AbilityFramework'))
        if not os.access(binary,os.X_OK):
            raise RuntimeError(f'AbilityFramework executable is unavailable: {binary}')
        with (cwd/'console.log').open('a') as log:
            process=subprocess.Popen([binary],cwd=cwd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (cwd/'started-by-yolo.json').write_text(json.dumps({'pid':process.pid}))
        for _ in range(30):
            try:existing=get('/api/config');break
            except urllib.error.URLError:time.sleep(.5)
        else:raise RuntimeError('Replay framework did not start')
    if 'yolo-replay-test' not in json.dumps(existing):
        raise RuntimeError('Port 8082 belongs to another framework; refusing to change it')
    try:get('/api/manifest/YoloDetect.Replay/1.0.0')
    except urllib.error.HTTPError as e:
        if e.code!=404:raise
        boundary='----yolo'+uuid.uuid4().hex
        archive=runtime/'yolo.detect.replay-1.0.0.zip'
        body=('--'+boundary+'\r\nContent-Disposition: form-data; name="file"; filename="yolo.detect.replay-1.0.0.zip"\r\nContent-Type: application/zip\r\n\r\n').encode()+archive.read_bytes()+('\r\n--'+boundary+'--\r\n').encode()
        request=urllib.request.Request(base+'/api/package',data=body,headers={'Content-Type':'multipart/form-data; boundary='+boundary})
        print(urllib.request.urlopen(request,timeout=10).read().decode(),flush=True)
    healthy=lambda:any(h.get('abilityName')=='YoloDetect.Replay' and h.get('state')=='Running' and h.get('abilityPort',0)>0 for h in get('/api/ability-heartbeat'))
    if not healthy():
        for _ in range(20):
            if any(c.get('metadata',{}).get('name')=='yolo-replay-test' for c in get('/api/cr')):break
            time.sleep(.5)
        request=urllib.request.Request(base+'/api/instance',data=json.dumps({'template':'yolo-replay-test','start':True,'connect':True}).encode(),headers={'Content-Type':'application/json'})
        print(urllib.request.urlopen(request,timeout=10).read().decode(),flush=True)
        for _ in range(45):
            if healthy():break
            time.sleep(1)
        else:raise RuntimeError('Replay Ability did not become ready; inspect framework console.log')
    print('READY: YoloDetect.Replay at '+base,flush=True)

if __name__=='__main__':main()
