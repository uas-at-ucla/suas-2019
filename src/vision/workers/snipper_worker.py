# import cv2 # cropping and saving images

from client_worker import ClientWorker


class SnipperWorker(ClientWorker):
    def __init__(self, in_q, socket_client, processes, args, verbose):
        super().__init__(in_q, socket_client, processes, args, verbose)
        self.real_data_dir = args.real_data_dir

    # task format:
    #   [{
    #       'src_img_id': str,
    #       'loc_info': {
    #           'lat': float,
    #           'lng': float,
    #           'alt': float
    #       },
    #       'yolo_results': [],
    #   }]
    def _do_work(self, task):
        print('Snipper called')
        src_img_id = task[0]['img_id']
        yolo_results = task[0]['yolo_results']

        src_img = self.manager.get_img(src_img_id)

        for result in yolo_results:
            xmin = result['topleft']['x']
            xmax = result['bottomright']['x']
            ymin = result['topleft']['y']
            ymax = result['bottomright']['y']

            # Crop the image
            # TODO calculate location
            cropped_img = src_img[ymin:ymax, xmin:xmax]
            img_id = self.manager.create_new_img(
                cropped_img,
                other={
                    'parent_img_id': src_img_id,
                    'location': {
                        'lat': result['coords']['lat'],
                        'lng': result['coords']['lng']
                    }
                })
            if self.verbose:
                print('Snipped {} -> {}'.format(src_img_id, img_id))

            self._emit(task, 'snipped', {
                'img_id': img_id,
                'download_dir': self.real_data_dir
            })

    def get_event_name(self):
        return 'snip'