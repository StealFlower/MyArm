#ifndef __OREBIN_H__
#define __OREBIN_H__

#include "board.h"

enum OreType : u16{
    nothing = 0,
    silvery = 100,
    gold = 300,
};

enum oreBinPos : u8{
    inTak = 0,
    inArm = 1,
    inThree = 2,
};

struct OreBin{
    public:
        u8 OreNum;
        struct OreProperty{
            OreType type;
        }*propertyOf;

        OreBin(uint8_t capacity);
        void addOre(uint8_t index,OreType type);
        void addOre(OreType type);
        void rmOre(uint8_t index);
        bool isFull(void);
        void clear(void);
    private:
        uint8_t capacity;
};

extern OreBin ore[];

#endif