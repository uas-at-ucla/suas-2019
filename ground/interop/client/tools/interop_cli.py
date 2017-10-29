#!/usr/bin/env python
# CLI for interacting with interop server.

from __future__ import print_function
import argparse
import datetime
import getpass
import logging
import pprint
import sys
import time

from interop import Client
from interop import Odlc
from interop import Telemetry
from proxy_mavlink import proxy_mavlink
from upload_odlcs import upload_odlcs

logger = logging.getLogger(__name__)


def missions(args, client):
    missions = client.get_missions()
    for m in missions:
        pprint.pprint(m.serialize())


def odlcs(args, client):
    if args.odlc_dir:
        upload_odlcs(client, args.odlc_dir, args.team_id,
                     args.actionable_override)
    else:
        odlcs = client.get_odlcs()
        for odlc in odlcs:
            pprint.pprint(odlc.serialize())


def probe(args, client):
    while True:
        start_time = datetime.datetime.now()

        telemetry = Telemetry(0, 0, 0, 0)
        telemetry_resp = client.post_telemetry(telemetry)
        obstacle_resp = client.get_obstacles()

        end_time = datetime.datetime.now()
        elapsed_time = (end_time - start_time).total_seconds()
        logger.info('Executed interop. Total latency: %f', elapsed_time)

        delay_time = args.interop_time - elapsed_time
        if delay_time > 0:
            try:
                time.sleep(delay_time)
            except KeyboardInterrupt:
                sys.exit(0)


def mavlink(args, client):
    proxy_mavlink(args.device, client)


def main():
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        stream=sys.stdout,
        format='%(asctime)s: %(name)s: %(levelname)s: %(message)s')

    # Parse command line args.
    parser = argparse.ArgumentParser(description='AUVSI SUAS Interop CLI.')
    parser.add_argument(
        '--url', required=True, help='URL for interoperability.')
    parser.add_argument(
        '--username', required=True, help='Username for interoperability.')
    parser.add_argument('--password', help='Password for interoperability.')

    subparsers = parser.add_subparsers(help='Sub-command help.')

    subparser = subparsers.add_parser('missions', help='Get missions.')
    subparser.set_defaults(func=missions)

    subparser = subparsers.add_parser(
        'odlcs',
        help='Upload odlcs.',
        description='''Download or upload odlcs to/from the interoperability
server.

Without extra arguments, this prints all odlcs that have been uploaded to the
server.

With --odlc_dir, this uploads new odlcs to the server.

This tool searches for odlc JSON and images files within --odlc_dir
conforming to the 2017 Object File Format and uploads the odlc
characteristics and thumbnails to the interoperability server.

There is no deduplication logic. Odlcs will be uploaded multiple times, as
unique odlcs, if the tool is run multiple times.''',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    subparser.set_defaults(func=odlcs)
    subparser.add_argument(
        '--odlc_dir',
        help='Enables odlc upload. Directory containing odlc data.')
    subparser.add_argument(
        '--team_id',
        help='''The username of the team on whose behalf to submit odlcs.
Must be admin user to specify.''')
    subparser.add_argument(
        '--actionable_override',
        help='''Manually sets all the odlcs in the odlc dir to be
actionable. Must be admin user to specify.''')

    subparser = subparsers.add_parser('probe', help='Send dummy requests.')
    subparser.set_defaults(func=probe)
    subparser.add_argument(
        '--interop_time',
        type=float,
        default=1.0,
        help='Time between sent requests (sec).')

    subparser = subparsers.add_parser(
        'mavlink',
        help='''Receive MAVLink GLOBAL_POSITION_INT packets and
forward as telemetry to interop server.''')
    subparser.set_defaults(func=mavlink)
    subparser.add_argument(
        '--device',
        type=str,
        help='pymavlink device name to read from. E.g. tcp:localhost:8080.')

    # Parse args, get password if not provided.
    args = parser.parse_args()
    if args.password:
        password = args.password
    else:
        password = getpass.getpass('Interoperability Password: ')

    # Create client and dispatch subcommand.
    client = Client(args.url, args.username, password)
    args.func(args, client)


if __name__ == '__main__':
    main()
