from datetime import datetime
from itertools import groupby
from enum import Enum
from dataclasses import dataclass


class SocType(Enum):
    INITIAL_SOC = "initial_soc"
    FINAL_SOC = "final_soc"

@dataclass
class IntervalSoc:
    interval_start: datetime
    interval_end: datetime
    soc_type: SocType
    prep_time: datetime
    soc_value: float
    
    
def get_latest_for_period(all_socs):
    initial_socs = filter(lambda x: x.soc_type == SocType.INITIAL_SOC, all_socs)
    initial_soc = max(initial_socs, key=lambda x: x.prep_time)
    final_socs = filter(lambda x: x.soc_type == SocType.FINAL_SOC, all_socs)
    final_soc = max(final_socs, key=lambda x: x.prep_time)
    return [initial_soc, final_soc]



def make_interval_list():
    
    interval_list = []
    
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,10,30),
            interval_end = datetime(2021,1,1,10,40),
            soc_type=SocType.FINAL_SOC,
            prep_time= datetime(2021,2,1,10,40),
            soc_value= 44,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,10,30),
            interval_end = datetime(2021,1,1,10,40),
            soc_type=SocType.INITIAL_SOC,
            prep_time= datetime(2021,2,1,10,40),
            soc_value= 45,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,10,30),
            interval_end = datetime(2021,1,1,10,40),
            soc_type=SocType.FINAL_SOC,
            prep_time= datetime(2021,3,1,10,40),
            soc_value= 46,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,10,30),
            interval_end = datetime(2021,1,1,10,40),
            soc_type=SocType.INITIAL_SOC,
            prep_time= datetime(2021,3,1,10,40),
            soc_value= 47,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,11,30),
            interval_end = datetime(2021,1,1,11,40),
            soc_type=SocType.FINAL_SOC,
            prep_time= datetime(2021,3,1,10,40),
            soc_value= 48,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,11,30),
            interval_end = datetime(2021,1,1,11,40),
            soc_type=SocType.INITIAL_SOC,
            prep_time= datetime(2021,3,1,10,40),
            soc_value= 49,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,11,30),
            interval_end = datetime(2021,1,1,11,40),
            soc_type=SocType.FINAL_SOC,
            prep_time= datetime(2021,4,1,10,40),
            soc_value= 50,
        )
    )
    interval_list.append(
        IntervalSoc(
            interval_start= datetime(2021,1,1,11,30),
            interval_end = datetime(2021,1,1,11,40),
            soc_type=SocType.INITIAL_SOC,
            prep_time= datetime(2021,4,1,10,40),
            soc_value= 51,
        )
    )
    
    return interval_list



def main():
    
    interval_list_raw = make_interval_list()
    
    # print(interval_list_raw[0])
    
    grouped = groupby(interval_list_raw, lambda x:x.interval_start)
    
    for key, val in grouped:
        print(key)
        
        a = get_latest_for_period(list(val))
        print(a)
        

    

    

if __name__ == "__main__":
    main()
