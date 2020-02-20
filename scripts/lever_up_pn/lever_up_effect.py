import keras
from DatasetLoader import MyDataset
import pickle
from Networks import *
from scipy.spatial.transform import Rotation as R

def find_bbox(shapes):
    
    corners=np.zeros((shapes.shape[0],shapes.shape[1],8,3))
    shape_info=shapes[:,:,7:10]/2
    cnt=0
    for i in [-1,1]:
        for j in [-1,1]:
            for k in [-1,1]:
                multiplier=np.array([i,j,k])
                corners[:,:,cnt,:]=shape_info*multiplier
                cnt=cnt+1
    corners_shape=corners.shape
    corners=corners.reshape(-1,8,3)
    quaternions=R.from_quat(shapes[:,:,3:7].reshape(-1,4))
    corners_transformed=np.zeros_like(corners)
    for i in range(8):
        corners_transformed[:,i,:]=quaternions.apply(corners[:,i,:])
    corners_transformed=corners_transformed.reshape(corners_shape)
    minxyz=np.min(corners_transformed,axis=2)
    maxxyz=np.max(corners_transformed,axis=2)
    bbox=maxxyz-minxyz
    return bbox

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

class Lever_Up_Effect:
    def __init__(self):
        with open('lev_up_scaler.pickle', 'r') as handle:
            scaler = pickle.load(handle)
        Pns= PropagationNetwork()
        Pn=Pns.getModel(7)    
        Pn.load_weights('lev_up_weights.hdf5')
        self.Pn=Pn
        self.lev_up_wall_trajs=MyDataset(PATH='trajectories/lev_up_wall/',
                                         n_of_traj=10,diff_ts=10,start_index=0,SCALER=scaler)
        self.lev_up_nowall_trajs=MyDataset(PATH='trajectories/lev_up_no_wall/',
                                         n_of_traj=10,diff_ts=10,start_index=0,SCALER=scaler)
    def predict_effect(self,dataset):
        PN=self.Pn
        traj_len=dataset.fr_size/dataset.diff_ts
        trajectories=np.zeros((dataset.n_of_traj,traj_len, 7,27))
        trajectories[:,:,:,:]=dataset.data[:,:,:,:].copy()
        trajectories[:,:,1:,20:]=0 # object motion are not known beforehand
        trajectories[:,1:,1,20:]=0 # pose not known
        trajectories[:,1:,1,13:]=0 # orientation not known
        for t in range(traj_len-1):
            input_to_Pn=dict()
            input_to_Pn['objects']=trajectories[:,t,:,:].copy()
            input_to_Pn['objects'][:,:,:3]= dataset.scaler_pos.transform(input_to_Pn['objects'][:,:,:3].reshape(-1,1)).reshape((dataset.n_of_traj, dataset.n_of_obj,3))
            input_to_Pn['objects'][:,:,13:16]= dataset.scaler_vel.transform(input_to_Pn['objects'][:,:,13:16].reshape(-1,1)).reshape((dataset.n_of_traj, dataset.n_of_obj,3))
            input_to_Pn['objects'][:,:,20:23]= dataset.scaler_vel.transform(input_to_Pn['objects'][:,:,20:23].reshape(-1,1)).reshape((dataset.n_of_traj, dataset.n_of_obj,3))
            input_to_Pn['objects'][:,:,7:13]= dataset.scaler_attri.transform(input_to_Pn['objects'][:,:,7:13].reshape(-1,6)).reshape((dataset.n_of_traj, dataset.n_of_obj,6))
            cnt = 0


            x_receiver_relations = np.zeros((dataset.n_of_traj, 7, 42), dtype=float);
            x_sender_relations   = np.zeros((dataset.n_of_traj, 7, 42), dtype=float);
            bbox = find_bbox(trajectories[:,t,:,:])
            for i in range(7):
                for j in range(7):
                    if(i != j):

                        asd = np.abs(trajectories[:,t,:,:][:,i,:3]-trajectories[:,t,:,:][:,j,:3])-(bbox[:,i,:3]+bbox[:,j,:3])/2<0.04
                        asd = np.logical_and(np.logical_and(asd[:,0],asd[:,1]),asd[:,2])
                        x_receiver_relations[asd, j, cnt] = 1.0
                        x_sender_relations[asd, i, cnt]   = 1.0
                        cnt += 1  
            input_to_Pn['sender_relations']=x_sender_relations
            input_to_Pn['receiver_relations']=x_receiver_relations
            input_to_Pn['relation_info']=dataset.r_i
            input_to_Pn['propagation']=np.zeros((dataset.n_of_traj, 7, 128), dtype=float);
            predicted_vel,predicted_quat = PN.predict(input_to_Pn)
            predicted_inv_transformed=np.zeros((dataset.n_of_traj, 1,7))
            predicted_inv_transformed[:,:,:3]=dataset.scaler_vel.inverse_transform(predicted_vel.reshape(-1,1)).reshape(dataset.n_of_traj, 1,3)
            predicted_inv_transformed[:,:,3:]=predicted_quat
            trajectories[:,t+1,1:2,:3]=trajectories[:,t,1:2,:3]+predicted_inv_transformed[:,:,:3]
            output_shape=trajectories[:,t+1,1:2,3:7].shape
            x_temp=R.from_quat(trajectories[:,t,1:2,3:7].reshape(-1,4))
            x_diff_temp=R.from_quat(predicted_quat.reshape(-1,4))
            trajectories[:,t+1,1:2,3:7]=(x_temp*x_diff_temp).as_quat().reshape(output_shape)#predicted_quat
            trajectories[:,t+1,1:2,13:20]=predicted_inv_transformed[:,:,:]
            for traj_ind in range(dataset.n_of_traj):
                trajectories[traj_ind,:t+2,1,3:7]=qfix(trajectories[traj_ind,:t+2,1,3:7]) 
        return trajectories
    def get_pcb_trajectory(self,pcb_size_x,pcb_size_y,direction_theta):
        dataset=self.lev_up_nowall_trajs
        lever_size_to_index=max(0,min(9,int((pcb_size_y-0.03)/0.005)))
        if np.abs(direction_theta+90) >45:
            dataset=self.lev_up_wall_trajs
            lever_size_to_index=max(0,min(6,int((pcb_size_x-0.05)/0.005)))

        print(lever_size_to_index)
        trajs=self.predict_effect(dataset)

        return trajs[lever_size_to_index,:,1,:3]