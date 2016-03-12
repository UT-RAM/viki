#!/usr/bin/python
import sys
import os

versionstart = 'VIKI_VERSION_INFO'
versionend = 'END_VERSION_INFO'
tempfile = 'bumpversion_temp_file'


def printUsageAndExit():
    print('Usage: bumpversion <version-number> <version-name>')
    sys.exit(1)


def processFile(fname):
    print('Processing file: ', fname)
    tf = open(tempfile, 'w')
    foundFlag = False
    try:
        with open(fname, 'r') as f:
            for line in f:
                if versionstart in line:
                    foundFlag = True
                    tf.write(line)
                    tf.write('\n')
                    tf.write('testtext')
                    tf.write('\n')
                    continue
                if versionend in line:
                    foundFlag = False
                    tf.write(line)
                    continue
                if not foundFlag:
                    tf.write(line)
                    continue
    finally:
        tf.close()
    print('Done!')


if __name__ == '__main__':
    if not len(sys.argv) is 3:
        printUsageAndExit()

    number = sys.argv[1]
    name = sys.argv[2]
    rootdir = os.path.dirname(os.path.realpath(__file__))
    exclude = ['.git']

    print('Updating version to number: ', number,
          'with name: ', name)
    print('Rootdir: ', rootdir)

    for root, dirs, files in os.walk(rootdir):
        dirs[:] = [d for d in dirs if d not in exclude]
        for file in files:
            with open(file, 'r') as f:
                for line in f:
                    if versionstart in line:
                        processFile(os.path.join(root, file))
