from collections import defaultdict, Counter
import numpy as np
import rospy 
from scipy.spatial.transform import Rotation as Rot
import itertools
import random

class Manipulation_Data:


    def get_all_workstations(self):
        all_workstations = rospy.get_param('/workstations/').keys()
        str_ws = []
        for i in range(len(all_workstations)):
            if all_workstations[i][0:6] == 'world2':
                str_ws.append(all_workstations[i][6:])
        return all_workstations

    """
    This algorithm simply groups the manipulation list by objects that have the same source and target location.
    In addition the result is is ordered in ascending group length.
    """
    def group_manipulation_list(self, obj_limit=3):
        sources = group_by(self.manipulation_list, 0)
        targets = group_by(self.manipulation_list, 1)
        # all_workstations = self.get_all_workstations()
        #for each in sources:
        #    if not(each) in sources:
        #        print(each)

        # get largest subset
        subsets = []
        for s in sources.values():
            for t in targets.values():
                if s and t:
                    subsets.append(intersection(s, t))

        subsets = sorted(subsets, key=len)
        subsets = delete_empty_indices(subsets) # delete empty indices from list
    
        # robot is limited to obj_limit=3 objects per task; therefore it may have to split tasks accordingly.
        i = 0
        while(i < len(subsets)): 
            if len(subsets[i]) > obj_limit: 
                print('unable to transport more than ' + str(obj_limit) + ' objects, splitting. ') # debug only
                while len(subsets[i]) > obj_limit:
                    k = 1
                    subsets.insert(i+k, subsets[i][obj_limit:obj_limit+3]) # insert movement task after current one
                    k += 1
                    del subsets[i][obj_limit:obj_limit+3] # delete the added tasks again
            i +=1
                
        # other strategy: only match source and then drive to several targets? 
    
        # last stretegy: get several objects and retrieve them to the same target 
        self.movements =  subsets
        

    def get_ws_distance(self, ws0, ws1): 
        r0 = self.WSPositions[ws0]
        r1 = self.WSPositions[ws1]
        return round((np.linalg.norm(r0 - r1)),2)   

    def get_ws_positions(self): 
        strs_all_ws = rospy.get_param('/workstations').keys()
        for ws in strs_all_ws:     
            if ws[0:6] == 'world2': 
                pose = rospy.get_param('/workstations/' + ws + '/robotPose')
                WS_trans = np.array(rospy.get_param('/workstations/' + pose + '/position'))
                WS_Rot = Rot.from_euler('xyz', rospy.get_param('/workstations/' + ws + '/orientation')).as_dcm() # rotation matrix of WS
                self.WSPositions[ws[6:]] = rospy.get_param('/workstations/' + ws + '/position') + np.dot(WS_Rot,WS_trans)
        self.WSPositions['ArenaExit'] = rospy.get_param('/workstations/ArenaExit/position')
        for keys in self.WSPositions.keys():
            self.WSPositions[keys][2] = 0

    # calculates the distance between the target-workstation of the current motion 
    # and source-workstation of the next motion; should be minimized
    def calculate_ws_distance(self): 
        print(self.movements)
        self.movement_distances = np.zeros(len(self.movements)-1)
        for i in range(len(self.movements)-1): 
#            manipulation_list[i] += [get_ws_distance(manipulation_list[i][0], manipulation_list[i][1])]
            self.movement_distances[i] = self.get_ws_distance(self.movements[i][-1][1], self.movements[i+1][0][0])
            
    
    def bruteForce_movements(self, n_max = 1e5): 
        n_move = len(self.movements) 
        permutations = list(itertools.permutations(range(n_move)))
        n_permute = len(permutations)
        self.order = permutations[0]
        return

        if n_permute > n_max: 
            # print('optimizing manipulation order via brute force, may take a short time.')# only use randomized sample?   
            n_permute  = int(n_max)
            permutations = random.sample(permutations, n_permute)
            
#        print('Optimizing ' + str(n_move) + ' movements by brute force.') # debug only
        self.costs = np.zeros(n_permute)
        for i in range(n_permute): 
            self.costs[i] += np.linalg.norm(self.WSPositions[self.movements[permutations[i][0]][0][0]]) # first WS of movement
            j = 0
            for j in range(n_move-1): 
                self.costs[i] += self.get_ws_distance(self.movements[permutations[i][j]][0][0], self.movements[permutations[i][j]][0][1]) # src current to current tar
                self.costs[i] += self.get_ws_distance(self.movements[permutations[i][j]][0][1], self.movements[permutations[i][j+1]][0][0]) # tar last to src next one
            self.costs[i] += self.get_ws_distance(self.movements[permutations[i][j]][0][1], self.movements[permutations[i][j+1]][0][0])
            r_lastTar = self.WSPositions[self.movements[permutations[i][-1]][0][1]] # last WS of movement  
            self.costs[i] +=  np.linalg.norm(np.array(self.WSPositions['ArenaExit']) - np.array(r_lastTar))
#        print('took ' + str(i) + 'iterations') # debug only 
        self.order = permutations[np.argmin(self.costs)]
        
    def sort_movements(self): 
        tmp_movements = list(self.movements)
        for i in range(len(tmp_movements)): 
            self.movements_opt.insert(i, tmp_movements[self.order[i]])
        self.movements = self.movements_opt
        del self.movements_opt
        
    def __init__(self, manipulation_list, move_obj_limit = 3):
        self.manipulation_list = manipulation_list
        # list of lists with the manipulation codes 
        # WS_src, WS_tar, object_ID)
        self.movements = []
        self.movement_distances = []
        self.WSPositions = dict()
        self.get_ws_positions()
        self.group_manipulation_list() 
        self.calculate_ws_distance() # distance between the target of last movement and source of next movement
        # try to optimize 
        self.movements_opt = []
        self.costs = []
        self.order = []
        self.bruteForce_movements()
        self.sort_movements()

# of two lists in most simple way
def intersection(lst1, lst2):
    lst3 = [value for value in lst1 if value in lst2]
    return lst3

def group_by(manipulation_list, index):
    groups = defaultdict(list)

    for obj in manipulation_list:
        groups[obj[index]].append(obj)

    new_list = groups.values()
    return groups

def list_duplicates_of(seq,item):
    start_at = -1
    locs = []
    while True:
        try:
            loc = seq.index(item,start_at+1)
        except ValueError:
            break
        else:
            locs.append(loc)
            start_at = loc
    return locs



def delete_empty_indices(subset): 
    for i in range(len(subset)-1, -1, -1): 
        if subset[i] == []: 
            subset.pop(i) # .append(subset[i])
    return subset

#def convert_to_list(subset, limit=3): 
##    for i in range(len(subset)): r
#        
#    return subset



if __name__=="__main__":
    
    possible_workstations = []
    for i in range(8): 
        possible_workstations +=['WS0' + str(i+1)]
    
    manipulation_list =[ # for testing
        ["WS01", "WS09", 11],
        ["WS01", "WS02", 12],
        ["WS08", "WS04", 15],
        ["WS03", "WS02", 16],
        ["WS01", "WS02", 16],
        ["WS01", "WS02", 16],
        ["WS03", "WS04", 16],
#        ["WS03", "WS04", 13],
#        ["WS05", "WS06", 14],
#        ["WS07", "WS08", 15],
#        ["WS02", "WS01", 16],
#        ["WS02", "WS01", 16],
#        ["WS02", "WS01", 16],
#        ["WS02", "WS01", 16],
#        ["WS02", "WS01", 16],
#        ["WS02", "WS01", 16],
#        ["WS02", "WS01", 16],
        ["WS02", "WS01", 16]
    ]
    manipulation_data = Manipulation_Data(manipulation_list)

    
    for i in range(len(manipulation_list)): 
        manipulation_list[i] += ['movement id ' + str(i)] 
        
        # fourth entry added as index of operation -> for administrating access 
#    mnp_list = np.array(manipulation_list)
#    np_sources = mnp_list[:,0]
#    np_targets = mnp_list[:,1]
#    n_manipulation = len(np_sources)
#    for i in range(n_manipulation): 
#        for j in range(i): 
#            if np_sources[i] == np_sources[j]: 
#                print(manipulation_list[i])

    sources = group_by(manipulation_list, 0)
    targets = group_by(manipulation_list, 1)
    # distance from the parameter server 
    
    
    
    
    # get largest subset
    subsets_both = []
    for s in sources.values():
        for t in targets.values():
            subsets_both.append(intersection(s, t))
#            temp_manipulation_list.pop(intersection(s,t))
#    subsets_sources = []
#    print('subset sources:')
    
#    for src in sources.keys(v): 
#        i
    

    
    
#    manipulation_subset = group_manipulation_list(manipulation_list)
#    print('Using fitting tar and dest needs ' +  str(len(manipulation_subset))+ ' movements')
#    
#    while(len(manipulation_subset) > 0): 
#        manipulation_subset = sorted(manipulation_subset, key=len)
#        print(manipulation_subset.pop())
        

