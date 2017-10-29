# Module to load odlcs from file and upload via interoperability.

import csv
import imghdr
import json
import logging
import os
import pprint
import re

from interop import Odlc

logger = logging.getLogger(__name__)

TARGET_TYPE_MAP = {
    'STD': 'standard',
    'OAX': 'off_axis',
    'EMG': 'emergent',
}

LATITUDE_REGEX = re.compile(
    '(?P<dir>[NS])(?P<deg>\d\d) (?P<min>\d\d) (?P<sec>\d\d\.\d{0,3})')
LATITUDE_DIR = {'S': -1, 'N': 1}
LONGITUDE_REGEX = re.compile(
    '(?P<dir>[EW])(?P<deg>\d\d\d) (?P<min>\d\d) (?P<sec>\d\d\.\d{0,3})')
LONGITUDE_DIR = {'E': -1, 'W': 1}


def load_odlc_file(odlc_filepath):
    """Loads odlcs from the given file.

    Args:
        odlc_filepath: The path to the odlc file to load.
    Returns:
        A list of (odlc, image_filepath) tuples.
    Raises:
        ValueError if the file is not properly formatted.
    """
    odlcs = []
    with open(odlc_filepath, 'r') as f:
        reader = csv.reader(f, delimiter='\t')
        for row in reader:
            odlc_type = row[1]
            latitude_str = row[2]
            longitude_str = row[3]
            orientation = row[4].lower()
            shape = row[5]
            background_color = row[6]
            alphanumeric = row[7]
            alphanumeric_color = row[8]
            image_filepath = row[9]
            description = row[10]

            odlc = Odlc()

            if odlc_type not in TARGET_TYPE_MAP:
                raise ValueError('Type %s not in %s' % (odlc_type,
                                                        str(TARGET_TYPE_MAP)))
            odlc.type = TARGET_TYPE_MAP[odlc_type]

            # Parse latitude. Not required for off axis.
            if odlc.type != 'off_axis':
                match = LATITUDE_REGEX.match(latitude_str)
                if match:
                    latitude = LATITUDE_DIR[match.group('dir')] * (
                        float(match.group('deg')) + float(match.group(
                            'min')) / 60 + float(match.group('sec')) / 3600)
                    odlc.latitude = latitude
                else:
                    raise ValueError(
                        'Latitude is not valid: %s' % latitude_str)

            # Parse longitude. Not required for off axis.
            if odlc.type != 'off_axis':
                match = LONGITUDE_REGEX.match(longitude_str)
                if match:
                    longitude = LONGITUDE_DIR[match.group('dir')] * (
                        float(match.group('deg')) + float(match.group(
                            'min')) / 60 + float(match.group('sec')) / 3600)
                    odlc.longitude = longitude
                else:
                    raise ValueError(
                        'Longitude is not valid: %s' % longitude_str)

            if orientation:
                odlc.orientation = orientation
            if shape:
                odlc.shape = shape
            if background_color:
                odlc.background_color = background_color
            if alphanumeric:
                odlc.alphanumeric = alphanumeric
            if alphanumeric_color:
                odlc.alphanumeric_color = alphanumeric_color
            if description:
                odlc.description = description
            odlcs.append((odlc, image_filepath))
    return odlcs


def upload_odlc(client,
                odlc_file,
                image_file,
                team_id=None,
                actionable_override=None):
    """Upload a single odlc to the server

    Args:
        client: interop.Client connected to the server
        odlc_file: Path to file containing odlc details in the Object
            File Format.
        image_file: Path to odlc thumbnail. May be None.
        team_id: The username of the team on whose behalf to submit odlcs.
            Defaults to None.
        actionable_override: Manually sets the odlc to be actionable. Defaults
            to None.
    """
    with open(odlc_file) as f:
        odlc = Odlc.deserialize(json.load(f))

    odlc.team_id = team_id
    odlc.actionable_override = actionable_override
    logger.info('Uploading odlc %s: %r' % (odlc_file, odlc))
    odlc = client.post_odlc(odlc)
    if image_file:
        logger.info('Uploading odlc thumbnail %s' % image_file)
        with open(image_file) as img:
            client.post_odlc_image(odlc.id, img.read())
    else:
        logger.warning('No thumbnail for odlc %s' % odlc_file)


def upload_odlcs(client, odlc_dir, team_id=None, actionable_override=None):
    """Upload all odlcs found in directory

    Args:
        client: interop.Client connected to the server
        odlc_dir: Path to directory containing odlc files in the Object
            File Format and odlc thumbnails.
        team_id: The username of the team on whose behalf to submit odlcs.
            Defaults to None.
        actionable_override: Optional. Overrides the odlc as actionable. Must
            be superuser to set.
    """
    odlcs = {}
    images = {}

    for entry in os.listdir(odlc_dir):
        name, ext = os.path.splitext(entry)

        if ext.lower() == '.json':
            if name in odlcs:
                raise ValueError(
                    'Found duplicate odlc files for %s: %s and %s' %
                    (name, odlcs[name], entry))
            odlcs[name] = os.path.join(odlc_dir, entry)
        elif ext.lower() in ['.png', '.jpg', '.jpeg']:
            if name in images:
                raise ValueError(
                    'Found duplicate odlc images for %s: %s and %s' %
                    (name, images[name], entry))
            images[name] = os.path.join(odlc_dir, entry)

    pairs = {}
    for k, v in odlcs.items():
        if k in images:
            pairs[v] = images[k]
        else:
            pairs[v] = None

    logger.info('Found odlc-image pairs:\n%s' % pprint.pformat(pairs))

    for odlc, image in pairs.items():
        upload_odlc(client, odlc, image, team_id, actionable_override)
