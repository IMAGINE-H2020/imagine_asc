#!/usr/bin/env python
import numpy as np 
import os
import skimage.io as io
import skimage.transform as trans
import keras
from keras.models import *
from keras.layers import *
from keras.optimizers import *
from keras import backend as keras
import rospkg
import cv2
class unetmodel:
    def __init__(self,model_name):

        config = tf.ConfigProto()

        config.gpu_options.allow_growth = True
        config.gpu_options.per_process_gpu_memory_fraction = 0.6

        self.session = tf.Session(config=config)

        keras.set_session(self.session)

        def unet(input_sizes,input_names,output_names,filter_sizes,target=(256,256),pretrained_weights=None):
            inputs_list=list()
            for input_ind in range(len(input_sizes)): 
                inputs_list.append(Input(input_sizes[input_ind],name=input_names[input_ind]))
            if len(inputs_list)==1:
                x=inputs_list[0]
            else:
                x =concatenate(inputs, axis = 3)
            convs=list()

            ## DOWNS
            for filter_index in range(len(filter_sizes)):
                conv = Conv2D(filter_sizes[filter_index], 3, activation = 'relu', padding = 'same',
                            kernel_initializer = 'he_normal')(x)
                conv = Conv2D(filter_sizes[filter_index], 3, activation = 'relu', padding = 'same',
                            kernel_initializer = 'he_normal')(conv)
                if filter_index>=len(filter_sizes)-2:
                    conv=Dropout(0.5)(conv)
                convs.append(conv)

                if filter_index==len(filter_sizes)-1:
                    x=conv
                else:
                    x = MaxPooling2D(pool_size=(2, 2))(conv)
                
            ## UPS
            for filter_index in range(len(filter_sizes)-2,-1,-1):
                up = Conv2D(filter_sizes[filter_index], 2, activation = 'relu',
                            padding = 'same', kernel_initializer = 'he_normal')(UpSampling2D(size = (2,2))(x))
                merge = concatenate([convs[filter_index],up], axis = 3)
                conv = Conv2D(filter_sizes[filter_index], 3, activation = 'relu',
                            padding = 'same', kernel_initializer = 'he_normal')(merge)
                x = Conv2D(filter_sizes[filter_index], 3, activation = 'relu',
                        padding = 'same', kernel_initializer = 'he_normal')(conv)
            x = Conv2D(2, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal')(x)    

            output = Conv2D(len(output_names), 1, activation = 'sigmoid')(x)
        
            model = Model(inputs = inputs_list, outputs = output)

            return model
        self.model = unet([(256,256,3)],
                    ['img'],
                    ['mask0'],
                    [64,128,256,512,1024])
        rospack = rospkg.RosPack()

        self.model.load_weights(os.path.join(rospack.get_path('imagine_asc'),'weights/'+model_name+'.hdf5'))
        self.model._make_predict_function()
    def predict(self,img):
        
        resized_image = cv2.resize(img, (256, 256))
        result = self.model.predict({'img':resized_image.reshape(-1,256,256,3)/255.0})

        aff_masks=result.reshape(256,256)*255
        aff_masks[aff_masks>100]=255
        aff_masks[aff_masks<151]=0
        return aff_masks.astype('uint8')
