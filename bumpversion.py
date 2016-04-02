#!/usr/bin/python
import sys
import os

versionstart = 'VIKI_VERSION_INFO'
versionend = 'END_VERSION_INFO'
tempfile = 'bumpversion_temp_file'
licensefile = 'MIT_LICENSE.txt'
grantfile = 'GRANT.txt'

try:
    with open(licensefile, 'r') as lf:
        lincensetext = lf.read()
except EnvironmentError:
    print("could not open license file ", licensefile)
    sys.exit()

try:
    with open(grantfile, 'r') as lf:
        granttext = lf.read()
except EnvironmentError:
    print("could not open grant file ", grantfile)
    sys.exit()


def printUsageAndExit():
    print('Usage: bumpversion <version-number> <version-name>')
    sys.exit(1)


def processFile(fname, versiontext):
    print('Processing file: ', fname)
    tf = open(tempfile, 'w')
    foundFlag = False
    try:
        with open(fname, 'r') as f:
            for line in f:
                if versionstart in line:
                    foundFlag = True
                    tf.write(line)
                    tf.write(versiontext)
                    tf.write(lincensetext)
                    tf.write('\n')
                    tf.write(granttext)
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
        os.rename(tempfile, fname)
    print('Done!')


if __name__ == '__main__':
    if not len(sys.argv) is 3:
        printUsageAndExit()

    number = sys.argv[1]
    name = sys.argv[2]
    versiontext = str.format('VIKI: more than a GUI for ROS \n\
version number: {}\n\
version name: {}\n\n', number, name)
    rootdir = os.path.dirname(os.path.realpath(__file__))
    exclude = ['.git']

    print('Updating version to number: ', number,
          'with name: ', name)
    print('Rootdir: ', rootdir)

    for root, dirs, files in os.walk(rootdir):
        dirs[:] = [d for d in dirs if d not in exclude]
        for file in files:
            if file.endswith('.pyc'):
                continue
            if file == 'bumpversion.py':
                continue
            if file == 'bumpversion_temp_file':
                continue
            with open(os.path.join(root, file), 'r') as f:
                for line in f:
                    if versionstart in line:
                        processFile(os.path.join(root, file), versiontext)
