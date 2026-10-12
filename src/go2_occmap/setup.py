from setuptools import setup

setup(name='go2_occmap', version='0.1.0', packages=['go2_occmap'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/go2_occmap']),
                  ('share/go2_occmap', ['package.xml'])],
      package_data={'go2_occmap': ['assets.json']},
      entry_points={'console_scripts': ['simulator = go2_occmap.sim_node:main',
                                       'mapper = go2_occmap.map_node:main',
                                       'monitor = go2_occmap.monitor_node:main',
                                       'explorer = go2_occmap.explore_node:main']})
