ROS workspace ini oluşturmanız ve python paketlerini kurmanız gerek. Tüm kurulum aşamaları bittikten sonra "setup.py ve package.xml" kısmında kod içi güncellemeler yaptım o dosyalarıda sadece güncelleme yaptığım kısımları paylaştım. Sizde kendi setup ve package dosyanızda bu değişiklikleri uygulayın.

You need to create the ROS workspace and install the python packages. After all the installation steps are completed, I made code updates in the "setup.py and package.xml" sections and shared only the parts I updated in those files. Apply these changes to your own setup and package files.

Çalıştırmadan önce robot başlangıç konumlarını aşağıdaki komutlarla belirleyebiliriz:
$ ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle1'}"
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 0.0, y: 0.0, theta: 0.0, name: 'turtle1'}"
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 4.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
