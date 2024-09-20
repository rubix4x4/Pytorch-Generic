Test1 = [1,1,1,2,2,3]
Test2 = [0,0,1,1,1,1,2,3,3]

class Solution(object):
    def removeDuplicates(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        CurrentVal = nums[0]

        PointI = 0
        PointJ = 0
        ValueCount = 0

        while PointJ < len(nums):
            if nums[PointJ] == CurrentVal and ValueCount < 2:
                # if 
                ValueCount = ValueCount + 1
                nums[PointI] == nums[PointJ]
                PointI = PointI + 1
                PointJ = PointJ + 1
            
            elif nums[PointJ] != CurrentVal:
                ValueCount = 1
                CurrentVal = nums[PointJ]
                nums[PointI] == nums[PointJ]
                PointI = PointI + 1
                PointJ = PointJ + 1

            else:
                PointJ = PointJ + 1

        return nums