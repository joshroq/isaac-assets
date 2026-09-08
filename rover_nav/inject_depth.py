import sys, yaml
src, dst = sys.argv[1], sys.argv[2]
data = yaml.safe_load(open(src))
def add(cm):
    p = data.get(cm,{}).get(cm,{}).get('ros__parameters',{})
    ol = p.get('obstacle_layer')
    if not isinstance(ol, dict): return
    s = ol.get('observation_sources','scan')
    names = s.split() if isinstance(s,str) else list(s)
    if 'depth_cloud' not in names: names.append('depth_cloud')
    ol['observation_sources'] = ' '.join(names)
    ol['depth_cloud'] = {'topic':'/oak/points','data_type':'PointCloud2','marking':True,
      'clearing':True,'min_obstacle_height':0.08,'max_obstacle_height':1.5,
      'obstacle_max_range':4.0,'obstacle_min_range':0.0,'raytrace_max_range':4.5,'raytrace_min_range':0.0}
for cm in ('local_costmap','global_costmap'): add(cm)
yaml.safe_dump(data, open(dst,'w'), sort_keys=False)
print('injected depth_cloud into', dst)
