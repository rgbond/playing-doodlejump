import numpy as np
import tensorflow as tf
from PIL import Image

class deploy(object):
    def __init__(self):
        self.bot = tf.placeholder("float", [None, 128, 80, 6])
        top = self.conv(self.bot, 5, 5, 64, 1, 1, padding='VALID', relu=False, name='conv_a5')
        top = self.conv(top, 1, 1, 32, 1, 1, relu=False, name='conv_a1')
        top = self.conv(top, 5, 5, 64, 2, 2, padding='VALID', name='conv_b5')
        top = self.conv(top, 1, 1, 32, 1, 1, relu=False, name='conv_b1')
        top = self.conv(top, 3, 3, 64, 1, 1, name='conv_c3')
        top = self.conv(top, 1, 1, 32, 1, 1, relu=False, name='conv_c1')
        top = self.conv(top, 5, 5, 64, 2, 2, padding='VALID', name='conv_d5')
        top = self.conv(top, 1, 1, 32, 1, 1, relu=False, name='conv_d1')
        top = self.conv(top, 5, 5, 64, 2, 2, padding='VALID', name='conv_e5')
        top = self.conv(top, 1, 1, 32, 1, 1, relu=False, name='conv_e1')
        top = self.fc(top, 200, name='fc1')
        self.top = self.fc(top, 5, relu=False, name='score')
        self.saver = tf.train.Saver()
        self.sess = tf.Session()
        self.y = tf.placeholder("float", [None, 5])
        self.cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=self.top,
                                                                           labels=self.y))
        self.optimizer = tf.train.AdamOptimizer(learning_rate=0.00001).minimize(self.cost)
        self.correct_prediction = tf.equal(tf.argmax(self.top, axis=0),
                                  tf.argmax(self.y, axis=0))
        self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32))
        init = tf.global_variables_initializer()
        self.sess.run(init)

    def save(self):
        self.saver.save(self.sess, self.snapshots) 

    def restore(self, prefix):
        self.saver.restore(self.sess, prefix) 
        self.snapshots = prefix

    def run(self, imgs):
        return self.sess.run(self.top, feed_dict={self.bot: imgs})

    def train(self, x, y):
        opt = self.sess.run(self.optimizer, feed_dict={self.bot: x, self.y: y})
        loss, acc = self.sess.run([self.cost, self.accuracy], feed_dict={self.bot:x, self.y: y})
        return(loss, acc)
        
    def conv(self, top, kh, kw, co, sh, sw, name, relu=True, padding='SAME', biased=True):
        ci = top.get_shape()[-1]
        with tf.variable_scope(name) as scope:
            weights = tf.get_variable('weights', [kh, kw, ci, co])
            top = tf.nn.conv2d(top, weights, [1, sh, sw, 1], padding=padding)
            if biased:
                biases = tf.get_variable('biases', [co])
                top = tf.nn.bias_add(top, biases)
            if relu:
                top = tf.nn.relu(top, scope.name)
            return top

    def fc(self, top, num_out, name, relu=True):
        with tf.variable_scope(name) as scope:
            input_shape = top.get_shape()
            if input_shape.ndims == 4:
                # The input needs to be flattened
                dim = 1
                for d in input_shape[1:].as_list():
                    dim *= d
                feed_in = tf.reshape(top, [-1, dim])
            else:
                feed_in, dim = (top, input_shape[-1].value)
            weights = tf.get_variable('weights', shape=[dim, num_out])
            biases = tf.get_variable('biases', [num_out])
            if (relu):
                top = tf.nn.relu_layer(feed_in, weights, biases, name=scope.name)
            else:
                top = tf.nn.xw_plus_b(feed_in, weights, biases, name=scope.name)
            return top

    def load_convert(self, data_path, session, ignore_missing=False):
        '''Load network weights from convert.py.
        data_path: The path to the numpy-serialized network weights
        session: The current TensorFlow session
        ignore_missing: If true, serialized weights for missing layers are ignored.
        '''
        data_dict = np.load(data_path).item()
        for op_name in data_dict:
            with tf.variable_scope(op_name, reuse=True):
                for param_name, data in data_dict[op_name].iteritems():
                    try:
                        var = tf.get_variable(param_name)
                        session.run(var.assign(data))
                    except ValueError:
                        if not ignore_missing:
                            raise

    def format_image(self, img):
        img = np.array(img, dtype=np.float32)
        mean = np.array([67.2, 103.3, 82.0])
        img -= mean
        img *= 0.00392157
        return img

    def format_images(self, img1, img2):
        img1 = self.format_image(img1)
        img2 = self.format_image(img2)
        imgs = np.concatenate((img1, img2), 2)
        imgs = imgs.reshape(1, 128, 80, 6)
        return imgs

    def load_image(self, im_fn):
        im = Image.open(im_fn)
        return self.format_image(im)

    def load_images(self, im1_fn, im2_fn):
        img1 = Image.open(im1_fn)
        img2 = Image.open(im2_fn)
        return self.format_images(img1, img2)
