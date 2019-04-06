from client_worker import ClientWorker


class RsyncWorker(ClientWorker):
    def __init__(self, in_q, socket_client, processes, args, verbose): 
        super().__init__(in_q, socket_client, processes, args, verbose)

    # task format: [{'prev': {}, 'next': {}, 'user': str, 'addr': str,
    #                'img_remote_src': str, 'img_local_dest': str}]
    def _do_work(self, task):
        task_args = task[0]
        if self.verbose:
            print('Called rsync with args: <' + '> <'.join(map(str, task)) +
                  '>')

        success = True
        for i in range(len(task_args['img_remote_src'])):
            remote = task_args['img_remote_src'][i]
            local = task_args['img_local_dest'][i]

            rsync_command = ('rsync -vz --progress -e "ssh -p 22 '
                             '-i {id_file} -o UserKnownHostsFile={hosts_file}"'
                             ' {user}@{ip}:{remote_path} {local_path}').format(
                                 id_file=task_args['ssh_id_path'],
                                 hosts_file=task_args['hosts_path'],
                                 user=task_args['user'],
                                 ip=task_args['addr'],
                                 remote_path=remote,
                                 local_path=local)
            if self.verbose:
                print('Spawning rsync: ' + rsync_command)
            if 0 != self.processes.spawn_process_wait_for_code(rsync_command):
                success = False
        if success:
            self._emit(
                task, 'download_complete', {
                    'saved_path': task_args['img_local_dest'],
                    'next': task_args['next']
                })
        else:
            self._emit(
                task, 'download_failed', {
                    'attempted_path': task_args['img_local_dest'],
                    'prev': task_args['prev']
                })

    def get_event_name(self):
        return 'rsync'
