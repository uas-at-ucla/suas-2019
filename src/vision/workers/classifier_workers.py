import os
import tensorflow as tf  # noqa: E402

from classifier import vision_classifier  # noqa: E402
from config import Config
from client_worker import ClientWorker


class ShapeClassifierWorker(ClientWorker):
    def __init__(self, in_q, socket_client, processes, args, verbose):
        super().__init__(in_q, socket_client, processes, args, verbose)
        self.model = vision_classifier.load_model(args.model_path)
        self.data_dir = Config.DOCKER_DATA_DIR.value

        # Keras w/ Tensorflow backend bug workaround
        # This is required when using keras with tensorflow on multiple threads
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

    # task format:
    #   [{
    #       'img_id': str,
    #   }]
    def _do_work(self, task):
        img_id = task[0]['img_id']
        self.manager.get_img(img_id)
        img = vision_classifier.shape_img(
            os.path.join(self.data_dir, img_id + '.jpg'))

        # Keras w/ Tensorflow backend bug workaround
        with self.graph.as_default():
            prediction = vision_classifier.predict_shape(self.model, img)
            if self.verbose:
                print('Classified {} as a {}'.format(img_id, prediction))
            self._emit(task, 'classified', {
                'img_id': img_id,
                'type': 'shape',
                'shape': prediction
            })

    def get_event_name(self):
        return 'classify_shape'


class LetterClassifierWorker(ClientWorker):
    def __init__(self, in_q, socket_client, processes, args, verbose):
        super().__init__(in_q, socket_client, processes, args, verbose)
        self.model = vision_classifier.load_model(args.model_path)
        self.data_dir = Config.DOCKER_DATA_DIR.value

        # Keras w/ Tensorflow backend bug workaround
        # This is required when using keras with tensorflow on multiple threads
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

    # task format:
    #   [{
    #       'img_id': str,
    #   }]
    def _do_work(self, task):
        img_id = task[0]['img_id']
        self.manager.get_img(img_id)
        img = vision_classifier.letter_img(
            os.path.join(self.data_dir, img_id + '.jpg'))

        # Keras w/ Tensorflow backend bug workaround
        with self.graph.as_default():
            prediction = vision_classifier.predict_letter(self.model, img)
            if self.verbose:
                print('Classified {} as a {}'.format(img_id, prediction))
            self._emit(task, 'classified', {
                'img_id': img_id,
                'type': 'letter',
                'letter': prediction
            })

    def get_event_name(self):
        return 'classify_letter'
