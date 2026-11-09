"""Control-task checks on an owned, completed YOLO invocation only."""
import json,time,urllib.request
from pathlib import Path
import yaml

def find_record(value):
    if isinstance(value,dict):
        if 'invocation_id' in value and 'task_name' in value and 'command_id' in value:return value
        for key in ('result','output','data','payload'):
            found=find_record(value.get(key))
            if found is not None:return found
    return None

def check_lifecycle(run_dir,root,evidence_dir,ability_name='YoloDetect.Demo'):
    events=[json.loads(line) for line in (run_dir/'events.jsonl').read_text().splitlines() if line.strip()]
    event=next(e['fields'] for e in events if e.get('event')=='action.terminal' and e['fields'].get('status')=='succeeded' and e['fields'].get('ability_name')==ability_name)
    instance,invocation=event['ability_instance_id'],event['invocation_id']
    profile=Path(__import__('os').environ.get('PROFILE',str(root/'../src/config/yolo-robot-deployment.yaml')))
    endpoint=yaml.safe_load(profile.read_text())['ability_framework']['endpoint'].rstrip('/')
    transcript=[]
    def post(path,data):
        req=urllib.request.Request(endpoint+'/api/ability/'+instance+path,data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=20) as response:result=json.load(response)
        transcript.append({'path':path,'request':data,'response':result})
        (evidence_dir/'lifecycle-responses.json').write_text(json.dumps(transcript,indent=2))
        return result
    def invoke(task_type):
        reply=post('/api/task/start',{'task_type':task_type,'input':{'invocation_id':invocation,'reason':'YOLO suite terminal-state idempotence check'}})
        deadline=time.monotonic()+20
        while find_record(reply) is None:
            task_id=reply.get('task_id') or reply.get('taskId')
            if not task_id:raise ValueError(f'Control response has no execution record or task_id: {reply}')
            if time.monotonic()>deadline:raise TimeoutError('Control task timed out')
            time.sleep(.1);reply=post('/api/task/status',{'task_id':task_id})
            if str(reply.get('status','')).lower() in ('failed','error','cancelled'):raise ValueError(str(reply))
        return find_record(reply)
    before=invoke(4)
    if before.get('invocation_id')!=invocation or before.get('task_name') not in ('yolo_detect','generate_point_cloud') or before.get('status')!='completed' or before.get('command_id'):
        raise ValueError('Control test requires our completed non-motion YOLO execution')
    after=invoke(5)
    if any(after.get(k)!=before.get(k) for k in ('invocation_id','task_name','command_id','status','result')):
        raise ValueError('StopExecution unexpectedly changed a completed YOLO result')
    return {'status':'passed','instance_id':instance,'invocation_id':invocation,
        'GetExecution':'passed','StopExecution':'passed (completed invocation remains completed)',
        'limitation':'Does not verify interruption of in-flight synchronous inference'}
