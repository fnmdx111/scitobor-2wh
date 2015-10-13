#! /usr/bin/env python3

from collections import defaultdict
import sys
import os

homework_title = sys.argv[1]
base_file = sys.argv[2]

base_fname = base_file.split('.')[0]
hw_name = '%s_team_22' % homework_title

hw_func = '''
function %s(r)
    %s(r);
end
'''

visited = defaultdict(bool)
visited['%s.m' % hw_name] = True

def pour(infp, infn, pf):
    hwfp('%% <concat.py>: concatenating %s ------->\n' % infn)
    for line in infp:
        if 'trplot2' in line:
            pass
        elif 'display' in line:
            pass
        else:
            pf(line.rstrip() + '\n')
    pf('\n')


with open('%s.m' % hw_name, 'w') as hwf:
    hwfpln = lambda x: print(x, file=hwf)
    hwfp = lambda x: print(x, file=hwf, end='')

    hwfp('%% This file is concatenated automatically by <concat.py>\n')
    hwfp(hw_func % (hw_name, base_fname))
    hwfpln('')

    with open(base_file, 'r') as rf:
        pour(rf, base_file, hwfp)
        visited[base_file] = True

    for r, _, fs in os.walk('.'):
        for f in fs:
            if not visited[f]:
                visited[f] = True

                if f.endswith('.m'):
                    with open(f, 'r') as fp:
                        pour(fp, f, hwfp)
        break

