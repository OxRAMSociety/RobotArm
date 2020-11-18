# Intro to Linux Command Line

Here are the key links for all the things to do and parts you'll need to get started with navigating Ubuntu while in your VirtualBox Virutal Machine

## Before the session
Please make sure you've completeed the aims of the last session and have a functioning Ubuntu Virtual Machine. If you've had issues with this process, don't worry! We can work through any issues at the start of this session. 

## In the session

We're going to cover the basics of naviating the terminal in your Ubuntu, specifically:

1. Naviating between folders 
    - cd \<folder path>
2. Opening files
    - cat \<file name>
3. Creating folder
    - mkdir \<folder name>
4. Creating files
    - nano \<file name>
5. Copy files
    - cp \<source file> \<target location>
6. Copy folders
    - cp -r \<source folder> \<target folder>
7. Deleting files
    - rm \<file name>
8. Deleting folders
    - rm -rd \<folder name>

If we work through that quickly, we may move on to some basic scripts:

5. Making a bash script
    - nano \<file name>.sh
    - Include "#!/bin/bash" as first line
    - Try a basic script like: echo "Hello World"
6. Assigning permissions to run
    - chmod a+x \<file name>.sh
7. Running the scripts
    - ./\<file name>.sh


## Future useful terminal instructions

https://help.ubuntu.com/community/Beginners/BashScripting