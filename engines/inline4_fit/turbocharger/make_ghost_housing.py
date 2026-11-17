# Split each turbo mesh into inner (opaque, original material) and outer (see-through) primitives.
import json,struct,sys,math,copy
src,dst=sys.argv[1],sys.argv[2]
g=json.load(open(src+'/scene.gltf')); b=bytearray(open(src+'/scene.bin','rb').read())
AY,AZ,RIN=0.624,0.946,0.052   # shaft axis (runs along x) and the radius of the rotating assembly
def acc(i):
    a=g['accessors'][i]; v=g['bufferViews'][a['bufferView']]
    off=v.get('byteOffset',0)+a.get('byteOffset',0)
    n={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4}[a['type']]
    fmt={5126:'f',5125:'I',5123:'H'}[a['componentType']]
    sz=struct.calcsize(fmt); st=v.get('byteStride',n*sz)
    return [struct.unpack_from('<'+fmt*n,b,off+k*st) for k in range(a['count'])]
def comps(P,I):
    key={}; rep=[key.setdefault(tuple(round(c,6) for c in v),i) for i,v in enumerate(P)]
    par=list(range(len(P)))
    def f(x):
        while par[x]!=x: par[x]=par[par[x]]; x=par[x]
        return x
    for t in range(0,len(I),3):
        a,b_,c=rep[I[t]],rep[I[t+1]],rep[I[t+2]]; par[f(a)]=f(b_); par[f(c)]=f(b_)
    cs={}
    for t in range(0,len(I),3): cs.setdefault(f(rep[I[t]]),[]).append(t//3)
    return list(cs.values())
ghost={"name":"housing_ghost","alphaMode":"BLEND","doubleSided":True,
       "pbrMetallicRoughness":{"baseColorFactor":[0.3662525955988395,0.3564001441459435,0.33245153634617935,0.25],
                               "metallicFactor":0.0,"roughnessFactor":0.650668649656837}}
g['materials'].append(ghost); GM=len(g['materials'])-1
report=[]
for mi,m in enumerate(g['meshes']):
    p=m['primitives'][0]; P=acc(p['attributes']['POSITION']); I=[x[0] for x in acc(p['indices'])]
    inner,outer=[],[]
    for tris in comps(P,I):
        vs=[P[I[3*t+j]] for t in tris for j in range(3)]
        r=max(math.hypot(v[1]-AY,v[2]-AZ) for v in vs)
        x0,x1=min(v[0] for v in vs),max(v[0] for v in vs)
        if x1-x0<0.001 and -0.317<=x0<=-0.262: r=1.0   # flat walls of the centre housing count as housing
        (inner if r<=RIN else outer).append(tris)
        report.append((mi,len(tris),'in' if r<=RIN else 'OUT',round(r*1000),round(min(v[0] for v in vs)*1000),round(max(v[0] for v in vs)*1000)))
    prims=[]
    for group,mat in ((inner,p.get('material')),(outer,GM)):
        idx=[I[3*t+j] for tris in group for t in tris for j in range(3)]
        if not idx: continue
        while len(b)%4: b.append(0)
        off=len(b); b+=struct.pack('<%dI'%len(idx),*idx)
        g['bufferViews'].append({"buffer":0,"byteOffset":off,"byteLength":4*len(idx),"target":34963})
        g['accessors'].append({"bufferView":len(g['bufferViews'])-1,"componentType":5125,"count":len(idx),"type":"SCALAR"})
        q=copy.deepcopy(p); q['indices']=len(g['accessors'])-1; q['material']=mat; prims.append(q)
    m['primitives']=prims
g['buffers'][0]['byteLength']=len(b)
open(dst+'/scene.bin','wb').write(b); json.dump(g,open(dst+'/scene.gltf','w'),indent=2)
for r in sorted(report): print(r)
