#!/usr/bin/env python3

import argparse
import configparser
import os
import signal
import sys
import subprocess


def execute(cmd, exit_on_error=False):
    """
    Executes cmd as subprocess and yields on each line.

    Also intercepts all signals and relays to child process.

    cmd: list of string
    """

    class State:  # global singleton
        signum = None  # only remember non-SIGCHLD signals

    ps = subprocess.Popen(cmd,
                          stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT,
                          universal_newlines=True)

    def signal_handler(sig, _):
        if sig != signal.SIGCHLD:  # No need to forward SIGCHLD to child
            State.signum = sig
            ps.send_signal(sig)

    # Intercept signals
    for i in [x for x in dir(signal) if x.startswith('SIG')]:
        try:
            signum = getattr(signal, i)
            signal.signal(signum, signal_handler)
        except BaseException:
            pass

    # Process outputs from child
    for stdout_line in iter(ps.stdout.readline, ''):
        yield stdout_line

    # Teardown
    ps.stdout.close()
    return_code = 0
    try:
        return_code = ps.wait()
    except ProcessLookupError:
        return_code = -1
    if return_code == signal.SIGCHLD or State.signum is not None:
        return_code = 0
    if exit_on_error and return_code != 0:
        sys.exit(return_code)
    return return_code


def parse_args():
    task_choices = ['build', 'setup', 'connect', 'bash', 'stop']

    parser = argparse.ArgumentParser()
    parser.add_argument('--test', dest='print_cmds_only',
                        help='Print commands but do not execute them [False]',
                        action='store_true')
    parser.add_argument('task', help='Do task', type=str,
                        choices=task_choices)
    parser.add_argument('--name', dest='CONTAINER_NAME',
                        help='Docker container name '
                        '[retrieved from config.ini]',
                        type=str, default='')
    parser.add_argument('--wifi_dev', dest='WIFI_DEV',
                        help='Device name for Wifi USB dongle '
                        '[retrieved from config.ini]',
                        type=str, default='')
    parser.add_argument('--drone_ap', dest='DRONE_AP',
                        help='Drone\'s access point name '
                        '[retrieved from config.ini]',
                        type=str, default='')
    parser.add_argument('--local_cmd_client_port', dest='LOCAL_CMD_CLIENT_PORT',
                        help='Local port bound to Drone\'s command server '
                        '[retrieved from config.ini]',
                        type=str, default='')
    parser.add_argument('--local_vid_server_port', dest='LOCAL_VID_SERVER_PORT',
                        help='Local port for video stream server from Drone '
                        '[retrieved from config.ini]',
                        type=str, default='')
    args = parser.parse_args()
    return args, parser


def merge_args(cfg, args):
    if 'Session' not in cfg:
        cfg['Session'] = {}
    for key in ('CONTAINER_NAME', 'WIFI_DEV', 'DRONE_AP',
                'LOCAL_CMD_CLIENT_PORT', 'LOCAL_VID_SERVER_PORT'):
        if not hasattr(args, key):
            continue
        val = getattr(args, key)
        if len(val) > 0:
            cfg['Session'][key] = val


def dispatcher():
    # Parse arguments
    args, parser = parse_args()

    # Load and update config settings
    cfg = configparser.ConfigParser()
    config_path = os.path.join(os.getcwd(), 'config.ini')
    cfg.read(config_path)
    merge_args(cfg, args)

    # Define task commands
    cmds = []

    if args.task == 'build':
        image_name = cfg['Setup']['IMAGE_NAME']
        cmds.append(['docker', 'build', '-t', image_name, '.'])

    elif args.task == 'setup':
        image_name = cfg['Setup']['IMAGE_NAME']
        container_name = cfg['Session']['CONTAINER_NAME']
        wifi_dev = cfg['Session']['WIFI_DEV']
        drone_ap = cfg['Session']['DRONE_AP']
        local_cmd_client_port = cfg['Session']['LOCAL_CMD_CLIENT_PORT']
        local_vid_server_port = cfg['Session']['LOCAL_VID_SERVER_PORT']

        cmds.append(['docker', 'run',
                     '-d', '--rm', '--privileged',
                     '--env', 'DRONE_AP='+drone_ap,
                     '--env', 'WIFI_DEV='+wifi_dev,
                     '--env', 'LOCAL_CMD_CLIENT_PORT='+local_cmd_client_port,
                     '--env', 'LOCAL_VID_SERVER_PORT='+local_vid_server_port,
                     '--name', container_name,
                     image_name])
        cmds.append(['./setup_network.sh', container_name, wifi_dev])

    elif args.task == 'connect':
        container_name = cfg['Session']['CONTAINER_NAME']
        cmds.append(['docker', 'exec', container_name, '/connect_drone.sh'])

    elif args.task == 'bash':
        container_name = cfg['Session']['CONTAINER_NAME']
        cmds.append(['docker', 'exec', '-it', container_name, '/bin/bash'])

    elif args.task == 'stop':
        container_name = cfg['Session']['CONTAINER_NAME']
        cmds.append(['docker', 'kill', container_name])

    else:
        print('Undefined task: [%s]' % args.task)
        parser.print_help()
        sys.exit(1)

    # Save config settings
    if not args.print_cmds_only:
        with open(config_path, 'w') as f:
            cfg.write(f)

    # For a single command, execute single command using os.execvp by spawning
    # new program and replacing current Python process
    if len(cmds) == 1:
        cmd = cmds[0]
        print('> '+' '.join(cmd)+'\n')
        if not args.print_cmds_only:
            sys.stdout.flush()
            sys.stderr.flush()
            os.execvp(cmd[0], cmd)

    # For multiple commands, execute using subprocess, while passing through
    # outputs upon each new line
    else:
        for cmd in cmds:
            print('> '+' '.join(cmd)+'\n')
            if not args.print_cmds_only:
                for line in execute(cmd, exit_on_error=True):
                    print(line, end='\r')


if __name__ == '__main__':
    dispatcher()
