%%%%%%%% TEMPLATE %%%%%%%%%%%%%%%
clear all;
close all;

destination_foder = "./mat/";
blacklist = ["gazebo","dynamic_reconfigure", "visual","rosout"];
whitelist = [];


%% Load Bag

 % Open Bag and select topic
 bagToMatConverter(whitelist, blacklist, false, destination_foder)

