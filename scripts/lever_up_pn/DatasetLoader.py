import os
import pickle
import numpy as np
from tqdm import tqdm
import numpy as np
from scipy.spatial.transform import Rotation as R
import json
from sklearn.preprocessing import StandardScaler,MinMaxScaler
def js_r(filename):
    with open(filename) as f_in:
        return(json.load(f_in))
def qfix(q):
    """
    Enforce quaternion continuity across the time dimension by selecting
    the representation (q or -q) with minimal distance (or, equivalently, maximal dot product)
    between two consecutive frames.
    
    Expects a tensor of shape (L, 4), where L is the sequence length and J is the number of joints.
    Returns a tensor of the same shape.
    """
    assert len(q.shape) == 2
    assert q.shape[-1] == 4
    
    result = q.copy()
    dot_products = np.sum(q[1:]*q[:-1], axis=1)
    mask = dot_products < 0
    mask = (np.cumsum(mask, axis=0)%2).astype(bool)
    result[1:][mask] *= -1
    return result
class MyDataset:
    def __init__(self,**kwargs):
        self.PATH= kwargs['PATH']
        self.n_of_obj = 7
        self.n_of_rel = self.n_of_obj*(self.n_of_obj-1)
        self.f_size = 7+ 3 + 3 + 3 +4 + 7# Pose + Shape + Obj_type + velocity of objects + Previous Quaternion + Control Pose
        self.num_of_rel_type = 1 # rigid connection
        self.fr_size = 950 
        self.diff_ts=kwargs['diff_ts']
        self.n_of_traj = kwargs['n_of_traj']
        self.traj_lengths = np.zeros((self.n_of_traj,))
        self.start_index = kwargs['start_index']
        self.data=np.zeros([self.n_of_traj,self.fr_size/self.diff_ts,self.n_of_obj,self.f_size])
        self.r_i=np.zeros([self.n_of_traj,self.n_of_rel,self.num_of_rel_type])
        def createObjectFromDict(asd,objtype):
            #obj=np.zeros(((len(asd['trajectory'])-1)/self.diff_ts+1,10))
            obj=np.zeros(((min(950,len(asd['trajectory']))-1)/self.diff_ts+1,10))
            obj[:,7:]=asd['shape']
            obj[:,7]=obj[:,7]
            obj[:,8]=obj[:,8]
            obj[:,9]=obj[:,9]

            for index in range(0,min(950,len(asd['trajectory'])),self.diff_ts):
                data=asd['trajectory'][index]
                obj[index/self.diff_ts,:3] =np.array(data['position'])
                obj[index/self.diff_ts,3:7] =np.array(data['orientation'])
            obj[:,3:7]=qfix(obj[:,3:7])
            rots_x_inv=R.from_quat(obj[:-1,3:7]).inv()
            rots_y=R.from_quat(obj[1:,3:7])
            rots_x_diff=(rots_x_inv*rots_y).as_quat()    
            return obj,rots_x_diff
        def getScene(scene_ind):
            data = js_r(self.PATH+'/'+str(self.start_index+scene_ind+1)+'.txt')   
            Shapes=list()
            obj_ind=0
            traj_len=(min(950,len(data['pcb']['trajectory']))-1)/self.diff_ts+1 #
            self.traj_lengths[scene_ind]=traj_len
            for ky in ['Hand','pcb','hdd']:
                if ky=='hdd':
                    hdd_part_index=0
                    for part in data[ky]:
                        self.data[scene_ind,:traj_len,obj_ind,:10],quat_diffs=createObjectFromDict(part,ky)
                        self.data[scene_ind,:traj_len,obj_ind,11]=1
                        self.data[scene_ind,1:traj_len,obj_ind,16:20]=quat_diffs
                        self.data[scene_ind,:traj_len-1,obj_ind,23:27]=quat_diffs
                        hdd_part_index=hdd_part_index+1
                        obj_ind=obj_ind+1
                else:
                    self.data[scene_ind,:traj_len,obj_ind,:10],quat_diffs=createObjectFromDict(data[ky],ky)
                    self.data[scene_ind,1:traj_len,obj_ind,16:20]=quat_diffs
                    self.data[scene_ind,:traj_len-1,obj_ind,23:27]=quat_diffs
                    if ky=='Hand':
                        self.data[scene_ind,:traj_len,obj_ind,10]=1
                    if ky=='pcb':
                        self.data[scene_ind,:traj_len,obj_ind,12]=1
                    obj_ind=obj_ind+1
            self.data[scene_ind,:traj_len,obj_ind:,3:7]=[0,0,0,1]
            self.data[scene_ind,traj_len-1:,:,3:7]=[0,0,0,1]            
            cnt=0
        map(getScene,tqdm(range(self.n_of_traj)))
        self.data[:,1:,:,13:16]=self.data[:,1:,:,:3]-self.data[:,:-1,:,:3]        
        self.data[:,0,:,16:20]=[0,0,0,1]
        #self.data[:,1:,:,16:20]=self.data[:,:-1,:,3:7]
        self.data[:,:-1,:,20:23]=self.data[:,1:,:,:3]-self.data[:,:-1,:,:3]
        #self.data[:,:-1,:,23:27]=self.data[:,1:,:,3:7]

        self.poss_diff=np.zeros([self.n_of_traj,self.fr_size/self.diff_ts,self.n_of_rel,3])
        cnt=0
        for i in range(self.n_of_obj):
            for j in range(self.n_of_obj):
                if(i != j):
                    self.poss_diff[:,:,cnt,:3]=self.data[:,:,i,:3]-self.data[:,:,j,:3]
                    cnt=cnt+1
        if 'SCALER' in kwargs.keys():
            self.scaler_pos = kwargs['SCALER'][0]
            self.scaler_orientation = kwargs['SCALER'][1]
            self.scaler_vel = kwargs['SCALER'][2]
            self.scaler_attri = kwargs['SCALER'][3]
        else:
            self.scaler_pos = StandardScaler()
            self.scaler_pos.fit(self.poss_diff.reshape(-1,1))
            self.scaler_orientation = StandardScaler()
            self.scaler_orientation.fit(self.data[:,:,:,3:7].reshape(-1,4))
            self.scaler_vel = StandardScaler()
            vels=self.data[:,:,:,13:16]
            self.scaler_vel.fit(vels.reshape(-1,1))
            self.scaler_attri = MinMaxScaler()
            self.scaler_attri.fit(self.data[:,:,:,7:13].reshape(-1,6))     
            self.scaler_transfer=list()
            self.scaler_transfer.append(self.scaler_pos )
            self.scaler_transfer.append(self.scaler_orientation) 
            self.scaler_transfer.append(self.scaler_vel )
            self.scaler_transfer.append(self.scaler_attri)

        