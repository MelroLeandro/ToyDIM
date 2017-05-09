---
layout:     post
title:      Jupyter
author:     Mellean
tags: 		post template Jupyter Notebook MATLAB
subtitle:  	Install Jupyter with a MATLAB kernel
category:  ToyDIM
---
<!-- Start Writing Below in Markdown -->

# Introduction

IPython is an interactive command-line interface to Python. Jupyter Notebook offers an interactive web interface to many languages, including MATLAB.

This article will walk you through setting up a server to run Jupyter Notebook as well as teach you how to connect to and use the notebook. Jupyter notebooks (or simply notebooks) are documents produced by the Jupyter Notebook app which contain both computer code (e.g. Python) and rich text elements (paragraph, equations, figures, links, etc.) which aid in presenting reproducible research.

By the end of this guide, you will be able to run Python 2.7 code using Ipython and Jupyter Notebook running on a remote server. For the purposes of this tutorial, Python 2 (2.7.x) is used since many of the data science, scientific computing, and high-performance computing libraries support 2.7 and not 3.0+.

You can also run online Jupyter without installing it from http://try.jupyter.org

# Installation:

you can install Jupyter via conda or pip.

1. Using Anaconda
First, make sure Anaconda is installed (see http://www.techinfected.net/2016/03/install-anaconda-ubuntu-linux.html
).

    conda install jupyter

2. Using Pip or pip3
First, make sure you have pip or pip3 installed. (sudo apt-get install python-pip)


    pip install jupyter
or

    pip3 install jupyter


# Running Jupyter Notebook

Enter this command and jupyter will automatically open up in your default browser.

    jupyter notebook

the default address is http://127.0.0.1:8888

get more information

    jupyter notebook --help

#  Installing Kernals

Installing Jupyter automatically installs the python kernel for installing other programming language kernels
(see https://github.com/jupyter/jupyter/wiki/Jupyter-kernels).

## A Jupyter kernel for MATLAB

This kernel requires Jupyter with Python 3.5+, and the MATLAB engine for Python R2016b+.

Install with python -mpip install imatlab (from PyPI) or python -mpip install git+https://github.com/imatlab/imatlab (from Github); then run python -mimatlab install to register the kernel spec. In the absence of administrator rights, the --user flag should be added to any of these commands.

To use it, run one of:


    $ jupyter notebook
    # In the notebook interface, select Matlab from the 'New' menu
    $ jupyter qtconsole --kernel imatlab
    $ jupyter console --kernel imatlab

