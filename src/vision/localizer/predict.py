from darkflow.net.build import TFNet
import cv2
import random
import signal
import sys
import time
import argparse
import inspect
import xml.etree.ElementTree as ET

DEFAULT_PB = "built_graph/yolo-auvsi.pb"
DEFAULT_META = "built_graph/yolo-auvsi.meta"
DEFAULT_TRESH = 0.0012


def signal_received(signal, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_received)


def randomColor():
    return (random.randint(0, 255), random.randint(0, 255),
            random.randint(0, 255))


def load_model(pb, meta, thresh):
    options = {"pbLoad": pb, "metaLoad": meta, "threshold": thresh}
    return TFNet(options)


def calc_dist_sqr(a, b):
    return (a[0] - b[0])**2 + (a[1] - b[1])**2


def check_lower_subset(truth, real):
    '''Returns True if the truth lower point is strictly greater than the real, i.e. if
    R is the real point, it returns true iif T (truth point) is within the shaded region:
            ^
            |
            |
       <----R########>
            #########>  <--- T should be in this region
            #########>
            #########>
            vvvvvvvvv
        '''
    for t_val in truth:
        for r_val in real:
            if t_val <= r_val:
                return False
    return True


def check_upper_subset(truth, real):
    '''Returns True if the truth lower point is strictly less than the real, i.e. if
    R is the real point, it returns true iif T (truth point) is within the shaded region:
        ^^^^^^^^^
       <#########
       <#########  <--- T should be in this region
       <#########
       <########R---->
                |
                |
                v
        '''
    for t_val in truth:
        for r_val in real:
            if t_val >= r_val:
                return False
    return True


def check_subset(truth, real):
    '''Returns True if the bounds of truth is within the bounds of real'''
    return check_lower_subset(truth[0], real[0]) and check_upper_subset(
        truth[1], real[1])


def run_test(args):
    n_missed = 0  # number missed
    mis_id = 0  # number of misidentifications
    total_dist = 0
    total_found = 0
    total = 0
    total_dupes = 0

    for img_path in os.listdir(args.target):
        if img_path.split('.')[1] == 'jpg':
            img = cv2.imread(os.path.join(args.target, img_path))
            results = tfnet.return_predict(img)
            root = ET.parse(
                os.path.join(args.target,
                             img_path.split('.')[0] + '.xml')).getroot()

            truth = root[4][4]
            truth_topleft = (truth[0], truth[1])
            truth_botright = (truth[2], truth[3])
            min_dist = None
            num_found = 0
            for result in results:
                topleft = (result['topleft']['x'], result['topleft']['y'])
                botright = (result['bottomright']['x'],
                            result['bottomright']['y'])

                if check_subset([truth_topleft, truth_botright],
                                [topleft, botright]):
                    avg_dist = (calc_dist_sqr(truth_topleft, topleft) +
                                calc_dist_sqr(truth_topleft, topleft)) / 2
                    if min_dist is None or avg_dist < min_dist:
                        min_dist = avg_dist
                    num_found += 1
                else:
                    mis_id += 1

            if min_dist is None:
                n_missed += 1
            else:
                total_dist += min_dist
                total_found += 1
                total_dupes += num_found - 1
            total += 1

    print(
        inspect.cleandoc('''\
        Results:
           Total Processed: {total}
           Found:           {percent_found:.2%}
           Avg Discrepancy: {pixel_off:.0f}
           Dupes/Found:     {precent_dupes:.2%}
           Miss-IDs/Found:  {precent_miss_id:.2%}'''.format(
            total=total,
            percent_found=total_found / total,
            pixel_off=total_dist / total_found,
            percent_dupes=total_dupes / total_found,
            percent_miss_id=mis_id / total_found)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pb', action='store', default=DEFAULT_PB)
    parser.add_argument('--meta', action='store', default=DEFAULT_META)
    parser.add_argument(
        '--thresh', action='store', type=float, default=DEFAULT_TRESH)
    parser.add_argument(
        '-d', dest='use_dir', action='store_true', default=False)
    parser.add_argument(
        'target',
        action='store',
        default=
        '/home/benlimpa/Projects/vision/target_generator/DATA/test_images/0000000.jpg'
    )

    args = parser.parse_args()

    if args.use_dir:
        run_test(args)
    else:
        tfnet = load_model(args.pb, args.meta, args.thresh)
        img = cv2.imread(args.target)
        results = tfnet.return_predict(img)

        for result in results:
            color = randomColor()
            topLeft = result['topleft']
            botRight = result['bottomright']
            coords = {
                'topLeft': (topLeft['x'], topLeft['y']),
                'botRight': (botRight['x'], botRight['y'])
            }
            cv2.putText(img, result['label'], coords['topLeft'],
                        cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
            cv2.rectangle(img, coords['topLeft'], coords['botRight'], color, 1)

        print("Results:")
        print(results)
        cv2.imshow("img", img)
        cv2.waitKey(2)
        while (True):
            time.sleep(0.5)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
