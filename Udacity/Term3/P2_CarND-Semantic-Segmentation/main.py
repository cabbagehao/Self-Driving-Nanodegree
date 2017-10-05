import os.path
import tensorflow as tf
import time
import helper
import warnings
from distutils.version import LooseVersion
from sklearn.utils import shuffle
import project_tests as tests
# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'
    
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()
    # Placeholder for input image.
    image_input = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3 = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4 = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7 = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return image_input, keep, layer3, layer4, layer7

tests.test_load_vgg(load_vgg, tf)

def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function

    # output:  
    #     1*1:  [2,5,18,2]


    # layer4:
    #     1*1   [2,10,36,2]

    # layer3:
    #     1*1   [2, 20, 72, 2]
    # conv 1*1
    initializer = tf.truncated_normal_initializer(stddev=0.01)


    conv_layer_7 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, padding='same',        
                                kernel_initializer=initializer)
    conv_layer_4 = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, padding='same',        
                                kernel_initializer=initializer)
    conv_layer_3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, padding='same',       
                                kernel_initializer=initializer)
                     

    # upsampling layer7 to match layer4
    decoder_layer_7 = tf.layers.conv2d_transpose(conv_layer_7, num_classes, 4, strides=(2, 2), padding='same',         
                                kernel_initializer=initializer) 
    output = tf.add(decoder_layer_7, conv_layer_4)


    # upsampling the sum to match layer3
    output = tf.layers.conv2d_transpose(output, num_classes, 4, strides=(2, 2), padding='same',         
                                kernel_initializer=initializer) 
    output = tf.add(output, conv_layer_3)


    # upsampling the sum to match original image size.
    output = tf.layers.conv2d_transpose(output, num_classes, 16, strides=(8, 8), padding='same',         
                                kernel_initializer=initializer) 
    
    return output

tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    correct_label = tf.reshape(correct_label, (-1, num_classes))
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=correct_label))

    train_step = tf.train.AdamOptimizer(learning_rate).minimize(cross_entropy_loss)

    return logits, train_step, cross_entropy_loss
tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    train_loss = [0, 0]
    for epoch in range(epochs):
        for X_train, y_train in get_batches_fn(batch_size):
            X_train, y_train = shuffle(X_train, y_train)
            _, loss = sess.run([train_op,cross_entropy_loss], feed_dict={input_image: X_train, correct_label: y_train, keep_prob: 0.8})
            print("_loss ", loss)
            train_loss[0] += loss
            train_loss[1] += len(X_train)

        print('loss: {:.2f}'.format(train_loss[0] / train_loss[1]))

tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    vedio_path = data_dir + "/test.avi"
    vedio_output = runs_dir + "/movie.mp4"
    epochs = 12
    batch_size = 10
    learning_rate = 0.001
    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    tests.test_for_kitti_dataset(data_dir)
    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/
    
    
    with tf.Session() as sess:

        with tf.device("/cpu:0"):
            # Path to vgg model
            vgg_path = os.path.join(data_dir, 'vgg')
            # Create function to get batches
            get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

            # OPTIONAL: Augment Images for better results
            #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network
            correct_label = tf.placeholder(tf.float32, [None, None, None, num_classes])
            keep_prob = tf.placeholder(tf.float32)

            input_image, keep_prob, layer3_out, layer4_out, layer7_out = load_vgg(sess, vgg_path)

            layer_output = layers(layer3_out, layer4_out, layer7_out, num_classes)
            logits, train_op, cross_entropy_loss = optimize(layer_output, correct_label, learning_rate, num_classes)
            # TODO: Train NN using the train_nn function
            sess.run(tf.global_variables_initializer())
            train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
                                   correct_label, keep_prob, learning_rate)
            helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)
            
            #saver.save(sess, './runs/saved/save.ckpt')

            # OPTIONAL: Apply the trained model to a video
            if os.path.exists(vedio_path):
                helper.gen_movie_output(sess, logits, keep_prob, input_image, vedio_path, vedio_output, image_shape)


if __name__ == '__main__':
    run()
    


