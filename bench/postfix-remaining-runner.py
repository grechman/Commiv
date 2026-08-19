#!/usr/bin/env python3
import os, subprocess, time, json, math, pathlib, glob, sys, traceback, fcntl
HOME=os.path.expanduser('~')
REPO=f'{HOME}/commiv-perf'
OUT=pathlib.Path(f'{HOME}/postfix-remaining')
OUT.mkdir(exist_ok=True)
RAW=OUT/'raw.log'; JSONL=OUT/'results.jsonl'; JOURNAL=OUT/'cells.done'; STATUS=OUT/'status.txt'; ERR=OUT/'errors.log'
BIN=f'{REPO}/zig-out/bin'
PY=f'{HOME}/cbench/venv/bin/python3'
VROOM=f'{HOME}/cbench/vroom/bin/vroom'
TW_DIR=f'{HOME}/commiv/vendor/road'
done=set(JOURNAL.read_text().splitlines()) if JOURNAL.exists() else set()
LOCK=(OUT/'runner.lock').open('w')
fcntl.flock(LOCK,fcntl.LOCK_EX|fcntl.LOCK_NB)
if JSONL.exists():
    for _line in JSONL.read_text().splitlines():
        try:
            _cell=json.loads(_line).get('_cell')
            if _cell:done.add(_cell)
        except Exception:pass

def status(label, extra=''):
    STATUS.write_text(f'{time.strftime("%F %T")} {label} {extra}\n')
    print(f'[{time.strftime("%F %T")}] {label} {extra}',flush=True)

def append_result(obj):
    with JSONL.open('a') as f: f.write(json.dumps(obj,sort_keys=True)+'\n')
    print('RESULT '+json.dumps(obj,sort_keys=True),flush=True)

def run(label,cmd,env=None,timeout=7200,cwd=REPO):
    e=os.environ.copy();e.update(env or {})
    status('START',label)
    t=time.time()
    try:
        p=subprocess.run(cmd,cwd=cwd,env=e,text=True,capture_output=True,timeout=timeout,shell=isinstance(cmd,str))
        out=(p.stdout or '')+(p.stderr or ''); rc=p.returncode
    except subprocess.TimeoutExpired as ex:
        out=(ex.stdout or '')+(ex.stderr or '')+'\nTIMEOUT';rc=124
    wall=time.time()-t
    with RAW.open('a') as f:f.write(f'\n==== {label} rc={rc} wall={wall:.3f}\n{out}\n')
    if rc != 0: raise RuntimeError(f'{label}: rc={rc}; tail={out[-500:]}')
    status('DONE',f'{label} wall={wall:.1f}s')
    return out,wall

def cell(cid,fn):
    if cid in done:
        return
    status('CELL',cid)
    try:
        objs=fn()
        if objs is None: objs=[]
        if isinstance(objs,dict): objs=[objs]
        for obj in objs:
            obj['_cell']=cid
            append_result(obj)
        with JOURNAL.open('a') as f:f.write(cid+'\n')
        done.add(cid)
    except Exception as ex:
        with ERR.open('a') as f:
            f.write(f'\n{time.strftime("%F %T")} {cid}\n{traceback.format_exc()}\n')
        status('FAILED',f'{cid}: {ex}')
        raise

def line_start(out,prefix):
    rows=[l.strip() for l in out.splitlines() if l.strip().startswith(prefix)]
    if len(rows)!=1: raise RuntimeError(f'expected one {prefix!r} row, got {len(rows)}')
    return rows[0]

def parallel(label,cmds,timeout):
    status('START',f'{label} ({len(cmds)} parallel)')
    ps=[subprocess.Popen(c,cwd=REPO,shell=True,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT) for c in cmds]
    outs=[];t=time.time()
    try:
        for p in ps:
            o,_=p.communicate(timeout=timeout);outs.append(o or '')
            if p.returncode: raise RuntimeError(f'{label}: rc={p.returncode}; tail={(o or "")[-500:]}')
    except Exception:
        for p in ps:
            if p.poll() is None:p.kill()
        raise
    with RAW.open('a') as f:
        f.write(f'\n==== {label} wall={time.time()-t:.3f}\n')
        for c,o in zip(cmds,outs):f.write(f'-- {c}\n{o}\n')
    status('DONE',f'{label} wall={time.time()-t:.1f}s')
    return outs

def build_and_guard():
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    if head!='26c8a1cca3f97e6d38498a968c6a4e786c8815ca': raise RuntimeError(f'wrong HEAD {head}')
    dirty=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True)
    if dirty: raise RuntimeError(f'dirty worktree: {dirty}')
    run('build-current-bench-binaries',['zig','build','moneyroadbench','pdptwbench','roadbench','twroadbench','vrptwbench','-Doptimize=ReleaseFast'],timeout=1200)

# Relevant to the NYC road-TW fleet/distance tradeoff: real-road PDPTW money objective.
def real_money():
    configs=[('moscow-100',30),('nyc-100',30),('moscow-1000',60),('nyc-1000',60),('moscow-2000',120),('nyc-2000',120)]
    for name,wall_s in configs:
        dump=OUT/f'money-real-{name}.json'
        def dumpfn(name=name,dump=dump):
            env={'MR_FILE':name,'MR_TW_DIR':TW_DIR,'MR_PAIR':'window','MR_SEED':'1','MR_TIMEPEN':'7','MR_VEH_PEN':'30000','MR_DUMP':str(dump),'MR_SOLVE':'0'}
            out,w=run(f'money-real/{name}/dump',['taskset','-c','0',f'{BIN}/commiv-moneyroadbench'],env,timeout=300)
            if not dump.exists():raise RuntimeError('dump missing')
            return {'family':'money_real','instance':name,'solver':'instance_dump','sha256':subprocess.check_output(['sha256sum',str(dump)],text=True).split()[0]}
        cell(f'money-real/{name}/dump',dumpfn)
        for drv in ('fleetmin','plain'):
            def cfn(name=name,wall_s=wall_s,drv=drv):
                env={'MR_FILE':name,'MR_TW_DIR':TW_DIR,'MR_PAIR':'window','MR_SEED':'1','MR_TIMEPEN':'7','MR_VEH_PEN':'30000','MR_DRIVER':drv,'MR_TIME_MS':str(wall_s*1000)}
                out,w=run(f'money-real/{name}/commiv/{drv}',['taskset','-c','0',f'{BIN}/commiv-moneyroadbench'],env,timeout=wall_s*3+300)
                row=line_start(out,name+',')
                return {'family':'money_real','instance':name,'solver':'commiv','driver':drv,'wall_budget':wall_s,'external_wall':w,'row':row}
            cell(f'money-real/{name}/commiv/{drv}',cfn)
        def vfn(name=name,wall_s=wall_s,dump=dump):
            env={'VROOM_BIN':VROOM,'VROOM_TIME':str(wall_s),'VROOM_THREADS':'1'}
            out,w=run(f'money-real/{name}/vroom',[PY,'tools/vroom_road_pdptw.py',str(dump)],env,timeout=wall_s*4+600)
            row=line_start(out,name+',')
            return {'family':'money_real','instance':name,'solver':'vroom','wall_budget':wall_s,'external_wall':w,'row':row}
        cell(f'money-real/{name}/vroom',vfn)

# Remaining quality-affecting large road CVRP cells.
def road_remaining():
    configs=[('moscow-2000',2000),('moscow-5000',5000),('berlin-2000',2000)]
    for name,dim in configs:
        rows=[]
        for seed in (1,2,3):
            def cfn(name=name,dim=dim,seed=seed):
                env={'RB_FILES':name,'RB_THREADS':'10','RB_SEED':str(seed),'RB_SYM':'0','RB_FINAL_LS':'1','RB_ROUTE_ATSP':'1'}
                if dim==2000:env.update(RB_ITERS='5000000',RB_MARATHON='1',RB_KICKS='50',RB_SUBSOLVE='50000',RB_SUBSOLVE_PAIRS='2')
                else:env.update(RB_ITERS='5000000',RB_MARATHON='1')
                out,w=run(f'road/{name}/commiv/s{seed}',['taskset','-c','0-9',f'{BIN}/commiv-roadbench'],env,timeout=2400)
                row=line_start(out,name+',')
                return {'family':'road','instance':name,'solver':'commiv','seed':seed,'external_wall':w,'row':row}
            cell(f'road/{name}/commiv/{seed}',cfn)
        recs=[json.loads(l) for l in JSONL.read_text().splitlines() if l.strip()]
        cr=[x for x in recs if x.get('family')=='road' and x.get('instance')==name and x.get('solver')=='commiv']
        if len(cr)!=3:raise RuntimeError(f'{name}: need 3 commiv rows, got {len(cr)}')
        wall=max(3,math.ceil(sum(float(x['row'].split(',')[-1]) for x in cr)/3/1000))
        def pfn(name=name,wall=wall):
            cmds=[f'{PY} tools/competitors/pyvrp_road.py {HOME}/commiv/vendor/road/{name}.road cvrp {wall} {s}' for s in (1,2,3)]
            outs=parallel(f'road/{name}/pyvrp',cmds,wall*3+600)
            objs=[]
            for s,o in zip((1,2,3),outs):objs.append({'family':'road','instance':name,'solver':'pyvrp','seed':s,'wall_budget':wall,'row':line_start(o,'pyvrp,')})
            return objs
        cell(f'road/{name}/pyvrp',pfn)

# Remaining quality-affecting road VRPTW n=2000 cells.
def roadtw_remaining():
    for name in ('moscow-2000','nyc-2000','berlin-2000'):
        for seed in (1,2,3):
            def cfn(name=name,seed=seed):
                env={'TP_FILE':name,'TP_TW_DIR':TW_DIR,'TP_ITERS':'800000','TP_THREADS':'10','TP_SEED':str(seed),'TP_COMBO':'1','TP_POLISH_EVERY':'8'}
                if name.startswith('nyc'):env['TP_NBR']='min'
                out,w=run(f'roadtw/{name}/commiv/s{seed}',['taskset','-c','0-9',f'{BIN}/commiv-twroadbench'],env,timeout=2400)
                return {'family':'roadtw','instance':name,'solver':'commiv','seed':seed,'external_wall':w,'row':line_start(out,name+',')}
            cell(f'roadtw/{name}/commiv/{seed}',cfn)
        recs=[json.loads(l) for l in JSONL.read_text().splitlines() if l.strip()]
        cr=[x for x in recs if x.get('family')=='roadtw' and x.get('instance')==name and x.get('solver')=='commiv']
        if len(cr)!=3:raise RuntimeError(f'{name}: need 3 commiv rows, got {len(cr)}')
        wall=max(3,math.ceil(sum(float(x['row'].split(',')[-1]) for x in cr)/3/1000))
        def pfn(name=name,wall=wall):
            cmds=[f'{PY} tools/competitors/pyvrp_road.py {HOME}/commiv/vendor/road/{name}.road vrptw {wall} {s}' for s in (1,2,3)]
            outs=parallel(f'roadtw/{name}/pyvrp',cmds,wall*3+600)
            return [{'family':'roadtw','instance':name,'solver':'pyvrp','seed':s,'wall_budget':wall,'row':line_start(o,'pyvrp,')} for s,o in zip((1,2,3),outs)]
        cell(f'roadtw/{name}/pyvrp',pfn)

# Remaining GH-1000 cells reached by native top-k.
def gh_remaining():
    for name in ('c2_10_1','r1_10_1','r2_10_1','rc1_10_1','rc2_10_1'):
        for seed in (1,2,3):
            def cfn(name=name,seed=seed):
                env={'VT_DIR':'vendor/vrptw/gh','VT_FILES':name,'VT_SEED':str(seed),'VT_ENGINE':'sisr','VT_ITERS':'2000000','VT_COMBO':'1'}
                out,w=run(f'gh/{name}/commiv/s{seed}',['taskset','-c','0-9',f'{BIN}/commiv-vrptwbench'],env,timeout=900)
                return {'family':'vrptw_gh','instance':name,'solver':'commiv','seed':seed,'external_wall':w,'row':line_start(out,name+',')}
            cell(f'gh/{name}/commiv/{seed}',cfn)
        recs=[json.loads(l) for l in JSONL.read_text().splitlines() if l.strip()]
        cr=[x for x in recs if x.get('family')=='vrptw_gh' and x.get('instance')==name and x.get('solver')=='commiv']
        if len(cr)!=3:raise RuntimeError(f'{name}: need 3 commiv rows')
        wall=max(3,math.ceil(sum(float(x['row'].split(',')[7]) for x in cr)/3/1000))
        def pfn(name=name,wall=wall):
            cmds=[f'{PY} {HOME}/campaign/scripts/pyvrp_solomon.py {REPO}/vendor/vrptw/gh/{name}.txt {wall} {s}' for s in (1,2,3)]
            outs=parallel(f'gh/{name}/pyvrp',cmds,wall*3+600)
            return [{'family':'vrptw_gh','instance':name,'solver':'pyvrp','seed':s,'wall_budget':wall,'row':line_start(o,'pyvrp,')} for s,o in zip((1,2,3),outs)]
        cell(f'gh/{name}/pyvrp',pfn)

def pd_instances():
    configs=[('100',pathlib.Path(REPO)/'vendor/pdptw',10,(1,2,3)),('200',pathlib.Path(REPO)/'vendor/pdptw/200',15,(1,)),('400',pathlib.Path(REPO)/'vendor/pdptw/400',30,(1,)),('600',pathlib.Path(REPO)/'vendor/pdptw/600',45,(1,)),('800',pathlib.Path(REPO)/'vendor/pdptw/800',60,(1,)),('1000',pathlib.Path(REPO)/'vendor/pdptw/1000',90,(1,))]
    for size,d,wall,seeds in configs:
        names=sorted(p.stem for p in d.glob('*.txt') if p.with_suffix('.sol').exists())
        for name in names:yield size,d,wall,seeds,name

# Full 352-cell synthetic money claim from README.
def money_full():
    for size,d,wall,seeds,name in pd_instances():
        rel=os.path.relpath(d,REPO)
        def cfn(size=size,rel=rel,wall=wall,name=name):
            env={'PB_DIR':rel,'PB_FILES':name,'PB_TIME_MS':str(wall*1000),'PB_THREADS':'1','PB_SEED':'1','PB_TIMEPEN':'1','PB_VEH_PEN':'280000'}
            out,w=run(f'money/{size}/{name}/commiv',['taskset','-c','0',f'{BIN}/commiv-pdptwbench'],env,timeout=wall*3+300)
            return {'family':'money','size':size,'instance':name,'solver':'commiv','seed':1,'wall_budget':wall,'external_wall':w,'row':line_start(out,name+',')}
        cell(f'money/{size}/{name}/commiv',cfn)
        def vfn(size=size,rel=rel,wall=wall,name=name):
            env={'VROOM_DIR':rel,'VROOM_BIN':VROOM,'VROOM_TIME':str(wall),'VROOM_THREADS':'1','VROOM_FIXED':'280000'}
            out,w=run(f'money/{size}/{name}/vroom',[PY,'tools/vroom_pdptw.py',name],env,timeout=wall*4+600)
            return {'family':'money','size':size,'instance':name,'solver':'vroom','wall_budget':wall,'external_wall':w,'row':line_start(out,name+',')}
        cell(f'money/{size}/{name}/vroom',vfn)

# Full 352-cell primary PDPTW quality campaign.
def pdptw_full():
    for size,d,wall,seeds,name in pd_instances():
        rel=os.path.relpath(d,REPO)
        for seed in seeds:
            def cfn(size=size,rel=rel,wall=wall,name=name,seed=seed):
                env={'PB_DIR':rel,'PB_FILES':name,'PB_TIME_MS':str(wall*1000),'PB_FLEET':'1','PB_EJECT':'1','PB_THREADS':'10','PB_SEED':str(seed)}
                if size!='100':env['PB_GRAN']='2'
                out,w=run(f'pdptw/{size}/{name}/commiv/s{seed}',['taskset','-c','0-9',f'{BIN}/commiv-pdptwbench'],env,timeout=wall*3+300)
                return {'family':'pdptw','size':size,'instance':name,'solver':'commiv','seed':seed,'wall_budget':wall,'external_wall':w,'row':line_start(out,name+',')}
            cell(f'pdptw/{size}/{name}/commiv/{seed}',cfn)
        def vfn(size=size,rel=rel,wall=wall,name=name):
            env={'VROOM_DIR':rel,'VROOM_BIN':VROOM,'VROOM_TIME':str(wall),'VROOM_THREADS':'10'}
            out,w=run(f'pdptw/{size}/{name}/vroom',[PY,'tools/vroom_pdptw.py',name],env,timeout=wall*4+600)
            return {'family':'pdptw','size':size,'instance':name,'solver':'vroom','wall_budget':wall,'external_wall':w,'row':line_start(out,name+',')}
        cell(f'pdptw/{size}/{name}/vroom',vfn)

def main():
    status('CAMPAIGN_START','current code only; no upgrades')
    cell('build',lambda:(build_and_guard() or {'family':'meta','solver':'commiv','commit':'26c8a1cca3f97e6d38498a968c6a4e786c8815ca'}))
    stages=[('money-real',real_money),('road',road_remaining),('roadtw',roadtw_remaining),('gh',gh_remaining),('money-full',money_full),('pdptw-full',pdptw_full)]
    for name,fn in stages:
        status('STAGE_START',name);fn();status('STAGE_DONE',name)
    status('CAMPAIGN_COMPLETE',f'{len(done)} cells')
if __name__=='__main__':
    main()
