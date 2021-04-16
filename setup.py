# -*- coding: utf-8 -*-

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='SPUdeS',
    version='2.0.0',
    description="SPUdeS is a six degrees of freedom Stewart Platform. This is an academic project by robotics engineering undergraduates at l'Université de Sherbrooke.",
    long_description=readme,
    url='https://github.com/SPUdeS/SPUdeS.git',
    license=license,
    packages=find_packages(exclude=('tests', 'docs', 'Documentation', 'CADs'))
)