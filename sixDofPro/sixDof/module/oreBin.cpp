#include "oreBin.h"

                //臂       取矿       三矿
OreBin ore[3]={OreBin(1),OreBin(1),OreBin(3)};
OreBin::OreBin(uint8_t capacity) : capacity(capacity){

    if(capacity > 0)
        propertyOf = (OreProperty*)malloc(sizeof(OreProperty)*capacity);
    for(uint8_t i=0;i<capacity;i++)
        propertyOf[i].type = nothing;

    OreNum=0;
}

void OreBin::addOre(uint8_t index,OreType type){
    if (index>=capacity || this->OreNum >=capacity)
        return;
    if(index < capacity){
        this->propertyOf[index].type = type;
        this->OreNum++;
    }
}

void OreBin::addOre(OreType type){
    if (this->OreNum >=capacity)
        return;
    this->propertyOf[OreNum].type = type;
    this->OreNum++;
}

void OreBin::rmOre(uint8_t index){
    if(OreNum <= 0 || index >= capacity)
        return;
    else{
        this->propertyOf[index].type = nothing;
        this->OreNum>0 ? this->OreNum-- : this->OreNum = 0;
    }
}

bool OreBin::isFull(){
    return this->OreNum >= this->capacity;
}

void OreBin::clear(){
    for(u8 i = 0;i < capacity;i++){
        this->propertyOf[i].type = nothing;
    }
    this->OreNum = 0;
}