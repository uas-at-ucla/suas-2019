from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

#Imports
import numpy as np
import tensorflow as tf
import cPickle as pickle

tf.logging.set_verbosity(tf.logging.INFO)

#application logic

def cnn_model_fn(features, labels, mode):

	#Input Layer
	input_layer = tf.reshape(features["x"], [-1, 32, 32, 1])

	#convolutional layer 1
	conv1 = tf.layers.conv2d(
		inputs=input_layer,
		filters=32,
		kernel_size=5,
		padding="same",
		activation=tf.nn.relu)

	#pooling layer 1
	pool1 = tf.layers.max_pooling2d(
		inputs=conv1, 
		pool_size=2, 
		strides=2)

	#convolutional layer 2
	conv2 = tf.layers.conv2d(
		inputs=pool1,
		filters=64,
		kernel_size=5,
		padding="same",
		activation=tf.nn.relu)

	#pooling layer 2
	pool2 = tf.layers.max_pooling2d(
		inputs=conv2,
		pool_size=2,
		strides=2)
	
	#Dense Layer
	pool2_flat = tf.reshape(pool2, [-1, 8 * 8 *64])
	dense = tf.layers.dense(
		inputs=pool2_flat,
		units=1024,
		activation=tf.nn.relu)
	dropout = tf.layers.dropout(
		inputs=dense,
		rate=0.4,
		training=mode == tf.estimator.ModeKeys.TRAIN)

	#logits layer
	logits = tf.layers.dense(inputs=dropout, units=27)

	predictions = {
		"classes": tf.argmax(input=logits, axis=1),
		"probabilities": tf.nn.softmax(logits, name="softmax_tensor")
	}
	if mode == tf.estimator.ModeKeys.PREDICT:
		return tf.estimator.EstimatorSpec(mode=mode, predictions=predictions)

	#caluclate loss
	onehot_labels = tf.one_hot(indices=tf.cast(labels, tf.int32), depth=27)
	#onehot_labels = labels
	loss = tf.losses.softmax_cross_entropy(
		onehot_labels=onehot_labels, logits=logits)

	#Configure training op
	if mode == tf.estimator.ModeKeys.TRAIN:
		optimizer = tf.train.GradientDescentOptimizer(learning_rate=0.001)
		train_op = optimizer.minimize(
			loss=loss,
			global_step=tf.train.get_global_step())
		return tf.estimator.EstimatorSpec(mode=mode, loss=loss, train_op=train_op)

	#evaluation metric
	eval_metric_ops = {
		"accuracy": tf.metrics.accuracy(
			labels=labels, predictions=predictions["classes"])}
	return tf.estimator.EstimatorSpec(
		mode=mode, loss=loss,
		eval_metric_ops=eval_metric_ops)

def main(unused_argv):
	#Load training and eval data
	train_data = np.array(pickle.load(open("train_values.p", "rb")), dtype=np.float32)
	train_labels = np.array(pickle.load(open("train_labels.p", "rb")))
	print(train_labels)
	eval_data = np.array(pickle.load(open("test_values.p", "rb")), dtype=np.float32)
	eval_labels = np.array(pickle.load(open("test_labels.p", "rb")))
	
	#Create esitmator
	letter_classifier = tf.estimator.Estimator(
		model_fn=cnn_model_fn, model_dir="/tmp/letter_convnet_model")

	# Set up logging for predictions
	tensors_to_log = {"probabilities": "softmax_tensor"}
	logging_hook = tf.train.LoggingTensorHook(
	      tensors=tensors_to_log, every_n_iter=1000)

	# Train the model
	train_input_fn = tf.estimator.inputs.numpy_input_fn(
	    x={"x": train_data},
	    y=train_labels,
	    batch_size=1,
	    num_epochs=None,
	    shuffle=True)
	letter_classifier.train(
	    input_fn=train_input_fn,
	    steps=10000,
	    hooks=[logging_hook])

	# Evaluate the model and print results
	eval_input_fn = tf.estimator.inputs.numpy_input_fn(
	    x={"x": eval_data},
	    y=eval_labels,
	    num_epochs=1,
	    shuffle=False)
	eval_results = letter_classifier.evaluate(input_fn=eval_input_fn)
	print(eval_results)

if __name__ == "__main__":
	tf.app.run()