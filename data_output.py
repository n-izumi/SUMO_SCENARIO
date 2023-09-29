import os

class DataOutput:
    
    def __init__(self, directory) -> None:
        self.directory =  directory


    def GuideTrafficLight(self, fileName):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write("#time, sim_time, system_state" + "\n")  

    def LeadingCar_LaneNumber(self,fileName):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write("#time, sim_time, node_no, status, car_number, distance, speed" + "\n")  

    def InsideCar(self, fileName):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write("#time, sim_time, inside_car_count" + "\n")  

    def StayCar_LaneNumber(self, fileName):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write("#time, sim_time, node_no, vehicle_count" + "\n")  

    def NearbyTrafficLight_LaneNumber(self, fileName):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write("#time, sim_time, node_no, color" + "\n")  

    def TrafficJam_LaneNumber(self, fileName):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write("#time, sim_time, node_no, vehicle_count" + "\n")  

    def write(self, fileName, datalist):
        with open(os.path.join(self.directory, fileName), 'a') as f:
            f.write(', '.join(datalist) + "\n")

if __name__ == "__main__":
    directory = "."
    dataOutPut = DataOutput(directory)