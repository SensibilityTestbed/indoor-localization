#!C:\Python27\python.exe
def search(finddata, expdata):
  if (len(finddata) > len(expdata)):
    return
  start = 0
  #Create a hash table of dict
  indexValues = [ {} for x in range(20)]
  for i in range(len(finddata)):
    index = binarySearch(finddata[i], expdata, start , len(expdata)-1)
    #hash data
    hashdata = int(finddata[i])
    #create more space on hash table if not enough space
    while (hashdata > len(indexValues)):
      for j in range(100):
        indexValues.append({})
     #add timestamp onto hash table
    indexValues[hashdata][finddata[i]] = index
    start = index + 1
  return indexValues
#A binary search algorithm to find the closest data value
def binarySearch(x, expdata, beginning, end):
  mid = int((end+beginning)/2)
  if expdata[mid] == x:
    return mid
  elif end - beginning <= 1:
    if x - expdata[beginning]  < expdata[end] - x:
      return beginning
    else:
      return end
  elif x < expdata[mid]:
    return binarySearch(x, expdata, beginning, mid)
  elif x > expdata[mid]:
    return binarySearch(x, expdata, mid+1, end)
def main():
    #Test Trial
    findList = [1.2, 19.27,19.65, 89]   
    list1 = [1,4,5,6,7,8,10,7,19.1, 19.2, 19.3, 19.65,56,89]
    indexValues = search(findList, list1)
    print ("Index of 19.27: " + str(indexValues[19][19.27]))
    print ("Index of 19.65: " + str(indexValues[19][19.65]))
    print ("Index of 1.2: " + str(indexValues[1][1.2]))
    print ("Index of 89: " + str(indexValues[89][89]))
if __name__ == '__main__':
    main()
