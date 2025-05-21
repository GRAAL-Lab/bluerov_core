Pull from json_utils repo on branch called "marco_branch"
create the build dir, and after compiled you'll obtain the executable udp_server

In test_package instead you have:
	- the test node, which actually is the udp_client which takes lat long innfo and publish them on two topics
	- the terminal node, which is the terminal you've to interact with

To run all you've to:

ros2 run test_package my_test
ros2 run test_package my_terminal
(from json_utils/build ./udp_server)

How to use:

From the terminal of udp_server select the message you wanna send
(you can choose whichever you want, they all send lat and long info with slightly different coordinates which can be changed only from server, i know
is quite pointless but I've not changed this script anymore since it has to be reworked from 0 given new specifications)

Then on the my_test terminal you should see that the node has received the info and is publishing them

Then on the my_terminal terminal you have to select the TBM you want to exec, type 1,2 or 3

After that, still in my_terminal, you can select on the menu the choice to make the service call
