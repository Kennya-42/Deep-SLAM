# python compute_bn_statistics.py ~/ENet/prototxts/enet_train_encoder_decoder.prototxt ~/ENet/weights/snapshots_decoder/enet_iter_44366.caffemodel ~/ENet/weights/weights_bn/ 

# python BN-absorber-enet.py 	--model ~/ENet/prototxts/enet_deploy_custom.prototxt --weights ~/ENet/weights/weights_bn/weights_bn.caffemodel --out_dir ~/ENet/final_model_weights/

python test_segmentation_video.py --model ~/ENet/final_model_weights/bn_conv_merged_model.prototxt --weights ~/ENet/final_model_weights/bn_conv_merged_weights.caffemodel --colours ~/ENet/scripts/cityscapes19.png --input_image ~/ENet/example_image/munich_000000_000019_leftImg8bit.png --out_dir ~/ENet/example_image/ --gpu 0
