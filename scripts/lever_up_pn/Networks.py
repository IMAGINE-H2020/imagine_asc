from keras.layers import Permute,Subtract,Add,Lambda,Input,Concatenate,TimeDistributed,Activation,Dropout,dot,Reshape
import tensorflow as tf
from keras.activations import tanh,relu
from keras import optimizers
from keras import regularizers
from Blocks import *
from keras import losses

def quat_loss(y_true, y_pred):
    losses=K.reshape(y_true-y_pred,(-1,4))
    new_loss=K.reshape(-y_true-y_pred,(-1,4))
    ol = K.sum(losses**2,axis=1)<=K.sum(new_loss**2,axis=1)
    nl = K.sum(losses**2,axis=1)>K.sum(new_loss**2,axis=1)
    return K.mean(K.square(K.concatenate([K.abs(losses[ol]),K.abs(new_loss[nl])], axis=0)),axis=-1)


class PropagationNetwork:
    def __init__(self):
        self.Nets={}
        self.set_weights=False
    def setModel(self,n_objects,PATH):
        self.Nets[n_objects].load_weights(PATH)
    def getModel(self,n_objects,object_dim=27,relation_dim=1):
        if n_objects in self.Nets.keys():
            return self.Nets[n_objects]
        n_relations  = n_objects * (n_objects - 1)
        #Inputs
        objects= Input(shape=(n_objects,object_dim),name='objects')

        sender_relations= Input(shape=(n_objects,n_relations),name='sender_relations')
        receiver_relations= Input(shape=(n_objects,n_relations),name='receiver_relations')
        permuted_senders_rel=Permute((2,1))(sender_relations)
        permuted_receiver_rel=Permute((2,1))(receiver_relations)
        relation_info= Input(shape=(n_relations,relation_dim),name='relation_info')
        
        ls1=128
        ls2=128
        ls3=128
        ls4=128
        propagation= Input(shape=(n_objects,ls4),name='propagation')
        
        # Getting sender and receiver objects
        senders=dot([permuted_senders_rel,objects],axes=(2,1))
        receivers=dot([permuted_receiver_rel,objects],axes=(2,1))
        
        # Getting specific features of objects for relationNetwork
        get_attributes=Lambda(lambda x: x[:,:,7:13], output_shape=(n_relations,6))
        get_z=Lambda(lambda x: x[:,:,2:3], output_shape=(n_relations,1))        
        get_pos=Lambda(lambda x: x[:,:,:3], output_shape=(n_relations,3))
        get_orientation=Lambda(lambda x: x[:,:,3:7], output_shape=(n_relations,4))
        get_vel=Lambda(lambda x: x[:,:,13:20], output_shape=(n_relations,7))
        get_control=Lambda(lambda x: x[:,:,20:27], output_shape=(n_relations,7))

        # Getting specific features of objects for objectNetwork        
        get_attributes2=Lambda(lambda x: x[:,:,7:13], output_shape=(n_objects,6))
        get_pose=Lambda(lambda x: x[:,:,3:7], output_shape=(n_objects,4))
        get_vel2=Lambda(lambda x: x[:,:,13:20], output_shape=(n_objects,7))
        get_control2=Lambda(lambda x: x[:,:,20:27], output_shape=(n_objects,7))
        get_z2=Lambda(lambda x: x[:,:,2:3], output_shape=(n_objects,1))        


        if(self.set_weights):
            rm = RelationalModel((n_relations,),53+relation_dim,[ls1,ls1,ls1],self.relnet,True)
            om = ObjectModel((n_objects,),18,[ls2,ls2],self.objnet,True)
            rmp = RelationalModel((n_relations,),ls4*2+ls1,[ls3,ls3],self.relnetp,True)
            omp = ObjectModel((n_objects,),ls2+ls3+ls4,[ls4,ls4],self.objnetp,True)
        else:
            rm=RelationalModel((n_relations,),53+relation_dim,[ls1,ls1,ls1])
            om = ObjectModel((n_objects,),18,[ls2,ls2])
            
            rmp=RelationalModel((n_relations,),ls4*2+ls1,[ls3,ls3])
            omp = ObjectModel((n_objects,),ls2+ls3+ls4,[ls4,ls4])
            
            self.set_weights=True
            self.relnet=rm.getRelnet()
            self.objnet=om.getObjnet()
            self.relnetp=rmp.getRelnet()
            self.objnetp=omp.getObjnet()
            
        r_att=get_attributes(receivers)
        s_att=get_attributes(senders)

        r_pos=get_pos(receivers)
        s_pos=get_pos(senders)
        
        r_orientation=get_orientation(receivers)
        s_orientation=get_orientation(senders)
        
        r_vel=get_vel(receivers)
        s_vel=get_vel(senders)
        
        r_control=get_control(receivers)
        s_control=get_control(senders)        

#        r_posvel = Concatenate()([r_pos,r_vel])
#        s_posvel = Concatenate()([s_pos,s_vel])
        
        # Getting dynamic state differences.
        dif_rs=Subtract()([r_pos,s_pos])
        
        # Creating Input of Relation Network
        #  3 + 2*7 +2*6 + 2*7 + 4*2 +2
        rel_vector_wo_prop= Concatenate()([relation_info,dif_rs,r_vel,s_vel,r_att,s_att,r_control,s_control,r_orientation,s_orientation,get_z(receivers),get_z(senders)])
        # 7 + 4 + 6 + 1
        obj_vector_wo_er=Concatenate()([get_vel2(objects),get_pose(objects),get_attributes2(objects),get_z2(objects)])   
        
        rel_encoding=Activation('relu')(rm(rel_vector_wo_prop))
        obj_encoding=Activation('relu')(om(obj_vector_wo_er))
       # rel_encoding=Dropout(0.1)(rel_encoding)
       # obj_encoding=Dropout(0.1)(obj_encoding)
        prop=propagation
       # prop_layer=Lambda(lambda x: x[:,:,7:], output_shape=(n_objects,256))

        for _ in range(1):
            senders_prop=dot([permuted_senders_rel,prop],axes=(2,1))
            receivers_prop=dot([permuted_receiver_rel,prop],axes=(2,1))
            rmp_vector=Concatenate()([rel_encoding,senders_prop,receivers_prop])
            x = rmp(rmp_vector)
            effect_receivers = Activation('relu')(dot([receiver_relations,x],axes=(2,1)))
            omp_vector=Concatenate()([obj_encoding,effect_receivers,prop])#
            x = omp(omp_vector)
            prop=Activation('relu')(Add()([x,prop]))
        x = Dense(7,kernel_regularizer=regularizers.l2(regul),
            bias_regularizer=regularizers.l2(regul),activation='linear')(prop) 
        
        velo=Lambda(lambda x: x[:,1:2,:3], output_shape=(1,3),name='target_vel')(x)
#        quat=Lambda(lambda x: x[:,1:2,3:7], output_shape=(1,4),name='target_quat')(x)
        quat=Lambda(lambda x: K.l2_normalize(x[:,1:2,3:7],axis=2), output_shape=(1,4),name='target_quat')(x)
#        target = Concatenate(name='target')([velo,quat])
        model = Model(inputs=[objects,sender_relations,receiver_relations,relation_info,propagation],outputs=[velo,quat])
        
        adam = optimizers.Adam(lr=0.001, epsilon=1e-7, decay=0.0, amsgrad=True)
        model.compile(optimizer=adam, loss=[losses.mse,losses.mae],loss_weights={'target_vel':1,'target_quat':1})
        self.Nets[n_objects]=model#quat_loss
        return model
class TemporalPropagationNetwork:
    def __init__(self):
        self.Nets={}
        self.set_weights=False
    def setModel(self,n_objects,PATH):
        self.Nets[n_objects].load_weights(PATH)
    def getModel(self,n_objects,object_dim=19,relation_dim=1,timestep_diff=100):
        if n_objects in self.Nets.keys():
            return self.Nets[n_objects]
        n_relations  = n_objects * (n_objects - 1)
        #Inputs
        objects= Input(shape=(timestep_diff,n_objects,object_dim),name='objects')

        sender_relations= Input(shape=(timestep_diff,n_objects,n_relations),name='sender_relations')
        receiver_relations= Input(shape=(timestep_diff,n_objects,n_relations),name='receiver_relations')
        permuted_senders_rel=Permute((1,3,2))(sender_relations)
        permuted_receiver_rel=Permute((1,3,2))(receiver_relations)
        relation_info= Input(shape=(timestep_diff,n_relations,relation_dim),name='relation_info')
        propagation= Input(shape=(timestep_diff,n_objects,100),name='propagation')
        
        # Getting sender and receiver objects
        senders=dot([permuted_senders_rel,objects],axes=(3,2))
        receivers=dot([permuted_receiver_rel,objects],axes=(3,2))
        
        # Getting specific features of objects for relationNetwork
        get_attributes=Lambda(lambda x: x[:,:,:,7:13], output_shape=(timestep_diff,n_relations,6))
        get_pos=Lambda(lambda x: x[:,:,:,:3], output_shape=(timestep_diff,n_relations,3))
        get_orientation=Lambda(lambda x: x[:,:,:,3:7], output_shape=(timestep_diff,n_relations,4))
        get_vel=Lambda(lambda x: x[:,:,:,13:16], output_shape=(timestep_diff,n_relations,3))
        get_control=Lambda(lambda x: x[:,:,:,16:19], output_shape=(timestep_diff,n_relations,3))

        # Getting specific features of objects for objectNetwork        
        get_attributes2=Lambda(lambda x: x[:,:,:,7:13], output_shape=(timestep_diff,n_objects,6))
        get_pose=Lambda(lambda x: x[:,:,:,:7], output_shape=(timestep_diff,n_objects,7))
        get_vel2=Lambda(lambda x: x[:,:,:,13:16], output_shape=(timestep_diff,n_objects,3))
        get_control2=Lambda(lambda x: x[:,:,:,16:19], output_shape=(timestep_diff,n_objects,3))
        
        if(self.set_weights):
            rm = RelationalModel((timestep_diff,n_relations,),32+relation_dim,[150,150,150,150],self.relnet,True)
            om = ObjectModel((timestep_diff,n_objects,),19,[100,100],self.objnet,True)
            rmp = RelationalModel((timestep_diff,n_relations,),350,[150,150,100],self.relnetp,True)
            omp = ObjectModel((timestep_diff,n_objects,),300,[100,100],self.objnetp,True)
            rme = RecurrentRelationalModel_many(n_objects,100,[100,7],self.rmepart1,self.rmepart2,True,timestep_diff,0)

        else:
            rm=RelationalModel((timestep_diff,n_relations,),32+relation_dim,[150,150,150,150])
            om = ObjectModel((timestep_diff,n_objects,),19,[100,100])
            
            rmp=RelationalModel((timestep_diff,n_relations,),350,[150,150,100])
            omp = ObjectModel((timestep_diff,n_objects,),300,[100,100])
            rme = RecurrentRelationalModel_many(n_objects,100,[100,7],None,None,False,timestep_diff,0)
            self.set_weights=True
            self.relnet=rm.getRelnet()
            self.objnet=om.getObjnet()
            self.relnetp=rmp.getRelnet()
            self.objnetp=omp.getObjnet()
            self.rmepart1,self.rmepart2=rme.getRelnet()
            
        r_att=get_attributes(receivers)
        s_att=get_attributes(senders)

        r_pos=get_pos(receivers)
        s_pos=get_pos(senders)
        
        r_orientation=get_orientation(receivers)
        s_orientation=get_orientation(senders)
        
        r_vel=get_vel(receivers)
        s_vel=get_vel(senders)
        
        r_control=get_control(receivers)
        s_control=get_control(senders)        

        r_posvel = Concatenate()([r_pos,r_vel])
        s_posvel = Concatenate()([s_pos,s_vel])
        
        # Getting dynamic state differences.
        dif_rs=Subtract()([r_posvel,s_posvel])
        
        # Creating Input of Relation Network
        rel_vector_wo_prop= Concatenate()([relation_info,dif_rs,r_att,s_att,r_control,s_control,r_orientation,s_orientation])
        obj_vector_wo_er=Concatenate()([get_vel2(objects),get_pose(objects),get_attributes2(objects),get_control2(objects)])   
        
        rel_encoding=Activation('relu')(rm(rel_vector_wo_prop))
        obj_encoding=Activation('relu')(om(obj_vector_wo_er))
        rel_encoding=Dropout(0.1)(rel_encoding)
        obj_encoding=Dropout(0.1)(obj_encoding)
        prop=propagation

        for _ in range(1):
            senders_prop=dot([permuted_senders_rel,prop],axes=(2,1))
            receivers_prop=dot([permuted_receiver_rel,prop],axes=(2,1))
            rmp_vector=Concatenate()([rel_encoding,senders_prop,receivers_prop])
            x = rmp(rmp_vector)
            effect_receivers = Activation('relu')(dot([receiver_relations,x],axes=(2,1)))
            omp_vector=Concatenate()([obj_encoding,effect_receivers,prop])#
            x = omp(omp_vector)
            prop=Activation('relu')(Add()([x,prop]))
        x=Permute((2,1,3))(prop)
        predicted=rme(x)
        start_step=lookstart
        prediction_mask= Input(shape=(n_objects,timestep_diff-start_step,7),name='prediction_mask')

        predicted=Lambda(lambda x:x[:,:,:,:]*prediction_mask,
                        output_shape =(n_objects,timestep_diff-start_step,7),name='target')(predicted)
        model = Model(inputs=[objects,sender_relations,receiver_relations,relation_info,propagation],outputs=[predicted])
        
        adam = optimizers.Adam(lr=0.0001)
        model.compile(optimizer=adam, loss='mse')
        self.Nets[n_objects]=model
        return model