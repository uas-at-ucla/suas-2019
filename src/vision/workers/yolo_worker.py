from darkflow.net.build import TFNet  # noqa: E402  # yolo neural net

from client_worker import ClientWorker


class YoloWorker(ClientWorker):
    def __init__(self, in_q, socket_client, processes, args, verbose):
        super().__init__(in_q, socket_client, processes, args, verbose)

        # load model
        yolo_options = {
            "pbLoad": args.yolo_pb,
            "metaLoad": args.yolo_meta,
            "threshold": args.yolo_threshold
        }
        self.tfnet = TFNet(yolo_options)

    # task format: [{'file_path': str}]
    def _do_work(self, task):
        img_id = task[0]['img_id']
        if self.verbose:
            print('Called yolo with args: <' + '> <'.join(map(str, task)) +
                  '>')

        img = self.manager.get_img(img_id)
        results = self.tfnet.return_predict(img)

        for result in results:
            result['confidence'] = float(result['confidence'])

        if self.verbose:
            print('yolo_results: ' + str(results))

        # TODO Calculate the coordinates
        #        for result in results:
        #            result
        self._emit(task, 'yolo_done', {'img_id': img_id, 'results': results})

        # Result format: [{'label': str, 'confidence': int,
        #                  'topleft': {'x': int, 'y': int},
        #              'bottomright': {'x': int, 'y': int},
        #                   'coords': {'lat': float, 'lng': float}}]

    def get_event_name(self):
        return 'yolo'


class MockYoloWorker(ClientWorker):
    def __init__(self, in_q, socket_client, args):
        super().__init__(in_q, socket_client, args)

    def get_event_name(self):
        return 'yolo'

    def _do_work(self, task):
        img_id = task[0]['img_id']
        img = self.manager.get_img(img_id)
        results = [{
            'label': 'rectangle',
            'confidence': 5,
            'topleft': {
                'x': img.shape[1] / 2 - 10,
                'y': img.shape[0] / 2 - 10
            },
            'bottomright': {
                'x': img.shape[1] / 2 + 10,
                'y': img.shape[0] / 2 + 10
            }
        }]
        self._emit(task, 'yolo_done', {'img_id': img_id, 'results': results})
