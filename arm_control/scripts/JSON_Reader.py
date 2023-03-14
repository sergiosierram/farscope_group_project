import json

f = open('work_order/test_pick_2.json')
data = json.load(f)


binsID = [] #bin_a bin_b
finalDict = {}
outputList = []
binContents = data["bin_contents"]

finalDict = list(data["bin_contents"].values())[1]
# print(finalDict)

workorder = data["work_order"]
# print(workorder[0])


##create list of bin IDs - bin_A ...
# for i in range(len(data["bin_contents"])):
#     print( (data["bin_contents"]).keys()  )
    
ids_list =   list ((data["bin_contents"]).keys()  ) 
# print("id: ", ids_list)


valsID = []
shelfID = 1
for i in range(len(data["work_order"])):
    vals = list( data["work_order"][i].values())
    binsID.append(vals[0])
    
    if vals[0] in ids_list:
        print(ids_list.index(vals[0]) + 1)
        shelfID = ids_list.index(vals[0]) + 1
    valsID.append(vals[1])
    outputList.append([shelfID, vals[1]])


# print(binsID)
# print(valsID)
print("\n")
print(outputList.index[0])
