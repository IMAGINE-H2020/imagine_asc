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

            x = Conv2D(len(output_names), 1, activation = 'sigmoid')(x)
            outputs_list=list()
            for output_ind in range(len(output_names)): 
                mask=Lambda(lambda x: x[:,:,:,output_ind:output_ind+1],
                            output_shape=(256,256,1),name=output_names[output_ind])(x)
                outputs_list.append(mask)

            model = Model(inputs = inputs_list, outputs = outputs_list)

            return model
        self.model = unet([(256,256,3)],
                    ['Orj_resized'],
                    ['Aff_mask'],
                    [64,128,256,512,1024])
        rospack = rospkg.RosPack()

        self.model.load_weights(os.path.join(rospack.get_path('test_image'),'weights/'+model_name+'.hdf5'))
        self.model._make_predict_function()
    def predict(self,img):
        
        resized_image = cv2.resize(img, (256, 256))
        result = self.model.predict({'Orj_resized':resized_image.reshape(-1,256,256,3)/255.0})

        Aff_masks=result.reshape(256,256)*255

        return Aff_masks.astype('uint8')
