all_cpus = ['amd64', 'raspi']

def cpu_select(values):
  for cpu in all_cpus:
    if cpu not in values:
      if 'else' in values:
        values[cpu] = values['else']
      else:
        fail('Need to handle %s CPUs!' % cpu, 'values')
  for key in values:
    if key not in all_cpus and key != 'else':
      fail('Not sure what a %s CPU is!' % key, 'values')
  return select({
    '//tools:cpu_k8': values['amd64'],
    '//tools:cpu_raspi': values['raspi'],
  })

def address_size_select(values):
  if '32' not in values:
    fail('Need to handle 32 bit addresses!', 'values')
  if '64' not in values:
    fail('Need to handle 64 bit addresses!', 'values')
  return select({
    '//tools:cpu_k8': values['64'],
    '//tools:cpu_roborio': values['32'],
  })

def compiler_select(values):
  if 'gcc' not in values:
    fail('Need to handle gcc!', 'values')
  if 'clang' not in values:
    fail('Need to handle clang!', 'values')
  return select({
    '//tools:compiler_gcc': values['gcc'],
    '//tools:compiler_clang': values['clang'],
  })
