from __future__ import print_function

import numpy as np 
import os
import skimage.io as io
import skimage.transform as trans
import keras
from keras.models import *
from keras.layers import *
from keras.optimizers import *
from keras import backend as keras
import cv2
from keras.preprocessing.image import ImageDataGenerator
import glob
from keras.callbacks import ModelCheckpoint, LearningRateScheduler

smooth = 1.

def dice_coef(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    return (2. * intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)

def dice_coef_loss(y_true, y_pred):
    return 1 - dice_coef(y_true, y_pred)


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

            output = Conv2D(len(output_names), 1, activation = 'sigmoid')(x)
        
            model = Model(inputs = inputs_list, outputs = output)

            return model
        self.model = unet([(256,256,1)],
                    ['mask'],
                    ['aff'],
                    [64,128,256,512,1024])
        self.model.compile(optimizer = Adam(lr = 3e-4), loss = dice_coef_loss, metrics = ['accuracy'])

def adjustData(img,mask):
    if(np.max(img) > 1):
        img = img / 255
        mask = mask /255
    img[img > 0.2] = 1
    img[img <= 0.2] = 0
    mask[mask > 0.2] = 1
    mask[mask <= 0.2] = 0
    return (img,mask)

def trainGenerator(batch_size,train_path,image_folder,mask_folder,aug_dict,image_color_mode = "grayscale",
                    mask_color_mode = "grayscale",image_save_prefix  = "image",mask_save_prefix  = "mask",
                    flag_multi_class = False,num_class = 2,save_to_dir = None,save_to_dir2 = None,target_size = (256,256),seed = 1):
    '''
    can generate image and mask at the same time
    use the same seed for image_datagen and mask_datagen to ensure the transformation for image and mask is the same
    if you want to visualize the results of generator, set save_to_dir = "your path"
    '''
    image_datagen = ImageDataGenerator(**aug_dict)
    mask_datagen = ImageDataGenerator(**aug_dict)
    image_generator = image_datagen.flow_from_directory(
        train_path,
        classes = [image_folder],
        class_mode = None,
        color_mode = image_color_mode,
        target_size = target_size,
        batch_size = batch_size,
        save_to_dir = save_to_dir,
        save_prefix  = image_save_prefix,
        seed = seed)
    mask_generator = mask_datagen.flow_from_directory(
        train_path,
        classes = [mask_folder],
        class_mode = None,
        color_mode = mask_color_mode,
        target_size = target_size,
        batch_size = batch_size,
        save_to_dir = save_to_dir2,
        save_prefix  = mask_save_prefix,
        seed = seed)
    train_generator = zip(image_generator, mask_generator)
    for (img,mask) in train_generator:
        img,mask = adjustData(img,mask)
        yield (img,mask)

data_gen_args = dict(rotation_range=50.0,
                    width_shift_range=0.2,
                    height_shift_range=0.2,
                    shear_range=0.15,
                    zoom_range=0.15,
                    horizontal_flip=True,
                    fill_mode='nearest')

myGenerator = trainGenerator(4,'cut/','mask','aff',data_gen_args)

cut_model = unetmodel("Cut")

model_checkpoint = ModelCheckpoint('unet_cut.hdf5', monitor='loss',verbose=1, save_best_only=True)

cut_model.model.fit_generator(myGenerator,steps_per_epoch=100,epochs=200,callbacks=[model_checkpoint])