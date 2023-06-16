
%Testing of FRS_loader_speed_change.m and FRS_Instance_speed_change.m
reachsets = loaders.RSLoader('converted_Au_frs.h5', do_caching=true, preload_data=false, preload_meta=false);
downstream_group = reachsets.getGroup(0);
arr_of_further_downstream_groups = reachsets.getGroup(0).getGroup();
collection_group = reachsets.getGroup(0);
terminal_group = collection_group.getGroup(0);

vehrs = reachsets.getGroup(0).getGroup(0).getZonos;
search_set = reachsets.getSearchSet();
search_set = reachsets.getSearchSet().getZonos();
zonos = terminal_group.getZonos();
z_matrix = zonos{1}.Z;
lat_speed = reachsets.getSearchSet().getZonos();
brake_idx1 = terminal_group.attributes.brake_index1;
brake_idx2 = terminal_group.attributes.brake_index2;
num_slicing = collection_group.num_groups;%slicing info 
frsLoader = FRS_loader_speed_change(vehrs,z_matrix);
