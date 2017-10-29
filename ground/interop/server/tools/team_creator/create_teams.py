"""Creates user accounts in the interop server and generates PDFs for teams.

This binary takes a csv file with team data. The first column should be the
team name. The second column should be the team username. For each team it
creates a random password. It then creates user accounts for each team. It then
prints Latex data that can be used to create a PDF to give to teams.
"""

import os
import sys
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "server.settings")
this_dir = os.path.dirname(os.path.abspath(__file__))
server_dir = os.path.join(this_dir, '..', '..', 'server')
sys.path = [server_dir] + sys.path
from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()

import argparse
import collections
import csv
import ipaddress
import random
import socket
import struct
from django.contrib.auth import get_user_model

CompetitionData = collections.namedtuple('CompetitionData', [
    'interop_ip', 'interop_hostname', 'interop_port', 'team_dhcp_range_min',
    'team_dhcp_range_max', 'team_static_range_min', 'team_static_range_max'
])

TeamData = collections.namedtuple(
    'TeamData', ['name', 'username', 'password', 'static_ip'])


def create_teams_data(teams_filepath, competition_data):
    """Creates user accounts and prints Latex for a PDF."""
    teams = get_team_info(teams_filepath,
                          competition_data.team_static_range_min,
                          competition_data.team_static_range_max)
    create_team_accounts(teams)
    print_latex_header()
    print_competition_data_latex(competition_data)
    print_team_datas_latex(teams)
    print_latex_footer()


def get_team_info(teams_filepath, static_min, static_max):
    """Gets the team information from the given file."""
    static_min_int = int(ipaddress.IPv4Address(unicode(static_min)))
    static_max_int = int(ipaddress.IPv4Address(unicode(static_max)))
    static_range = static_max_int - static_min_int

    random.seed()
    teams = []
    cur_static = 0
    with open(teams_filepath) as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            name = row[0]
            username = row[1]
            password = random.randint(1e9, 1e10)  # Random numeric password.
            static_ip_int = static_min_int + (cur_static % static_range)
            static_ip = str(ipaddress.IPv4Address(static_ip_int))
            team = TeamData(name, username, password, static_ip)
            teams.append(team)
            cur_static += 1
    return teams


def create_team_accounts(teams):
    """Creates team accounts."""
    for team in teams:
        get_user_model().objects.create_user(
            username=team.username, password=team.password)


def print_latex_header():
    """Prints the Latex header."""
    print '\\input{teams_data_base}'
    print '\\begin{document}'


def print_competition_data_latex(data):
    """Prints the competition data."""
    print('\\competition{%s}{%s}{%d}{%s}{%s}' %
          (data.interop_ip, data.interop_hostname, data.interop_port,
           data.team_dhcp_range_min, data.team_dhcp_range_max))


def print_team_datas_latex(teams):
    """Prints the team data."""
    for team in teams:
        print('\\team{%s}{%s}{%s}{%s}' % (team.name, team.username,
                                          team.password, team.static_ip))


def print_latex_footer():
    """Prints the Latex footer."""
    print '\\end{document}'


def main():
    """Configure the create team binary."""
    # Get parameters from command line.
    parser = argparse.ArgumentParser(
        description='Creates team accounts and generates Latex.')
    parser.add_argument(
        'teams_filepath',
        metavar='T',
        type=str,
        help='Filepath to csv file with col 0 of name, col 1 of username.')
    parser.add_argument(
        'interop_ip', metavar='I', type=str, help='Interop server IP address.')
    parser.add_argument(
        'interop_hostname',
        metavar='H',
        type=str,
        help='Interop server hostname.')
    parser.add_argument(
        'interop_port', metavar='P', type=int, help='Interop server port.')
    parser.add_argument(
        'team_dhcp_range_min',
        metavar='DL',
        type=str,
        help='Team DHCP range lower bound.')
    parser.add_argument(
        'team_dhcp_range_max',
        metavar='DU',
        type=str,
        help='Team DHCP range upper bound.')
    parser.add_argument(
        'team_static_range_min',
        metavar='SL',
        type=str,
        help='Team static IP range lower bound.')
    parser.add_argument(
        'team_static_range_max',
        metavar='SU',
        help='Team static IP range upper bound.')
    args = parser.parse_args()

    teams_filepath = args.teams_filepath
    interop_ip = args.interop_ip
    interop_hostname = args.interop_hostname
    interop_port = args.interop_port
    team_dhcp_range_min = args.team_dhcp_range_min
    team_dhcp_range_max = args.team_dhcp_range_max
    team_static_range_min = args.team_static_range_min
    team_static_range_max = args.team_static_range_max

    # Validate parameters.
    if not teams_filepath or not os.path.exists(teams_filepath):
        print 'Invalid teams filepath.'
        return
    if not interop_ip or not interop_hostname or not interop_port:
        print 'Invalid interop server details.'
        return
    if (not team_dhcp_range_min or not team_dhcp_range_max or
            not team_static_range_min or not team_static_range_max):
        print 'Invalid router details.'
        return

    # Create teams and print Latex data.
    competition_data = CompetitionData(
        interop_ip, interop_hostname, interop_port, team_dhcp_range_min,
        team_dhcp_range_max, team_static_range_min, team_static_range_max)
    create_teams_data(teams_filepath, competition_data)


if __name__ == '__main__':
    main()
