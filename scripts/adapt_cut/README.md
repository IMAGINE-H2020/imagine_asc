# Instructions
Replace images in mask folder with new cable masks, and replace images in aff folder with their corresponding affordance map (Pixelwise).

Note that each mask and affordace image pair should have the same file name in their corresponding folders.

Run train_cut.py. It will procude unet_cut.hdf5 which can be copied to weights folder to update cut model weights. Number of epochs is set to 200, but you can Ctrl+C the scripts early if accuracy/loss stops improving. The scripts saves the best weights.