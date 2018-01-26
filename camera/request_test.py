import requests
import json

# general lists and dictionaries I'll need later on. Final_dict has the general format already initialized.
general_list = []
roots = []
final_dict = {"valid_menus": [], "invalid_menus": []}

# gets the total number of pages for which the for loop can work through
URLx = "https://backend-challenge-summer-2018.herokuapp.com/challenges.json?id=1&page="
x = requests.get(url = URLx)
datax = x.json()
pages = datax['pagination']['total']

# loop through the web pages
for i in range(1, pages + 1):
	# get the web. I hardcoded this URL for simplicity. Do note that you can access the challenge URL by changing id = 2.
	URL = "https://backend-challenge-summer-2018.herokuapp.com/challenges.json?id=1&page=" + str(i)
	r = requests.get(url = URL)
	data = r.json()

	# loop through the dictionaries in menu
	for x in data['menus']:
		# create a temporary dict for each id
		a_dict = {str(x['id']): x["child_ids"]}

		# this looks for root nodes since they wouldn't have parent_id
		if 'parent_id' not in x:
			# append the dicts of the roots to a general list to be sorted later. Also append the root id to a list in order to be remembered
			general_list.append(a_dict)
			roots.append(str(x['id']))
		# the rest of the ids get sorted through here
		else:
			for y in general_list:
				# sorts the ids based on who their parents are. Since there are three roots, there will be three trees and thus all of the other ids must belong to one of the three trees
				if str(x['parent_id']) in y:
					# creates keys for each of the nodes in the trees
					y[str(x['id'])] = x['child_ids']
					for child in x['child_ids']:
						# this logic sorts whether the tree is valid or invalid. The tree is invalid if a child is also a parent. Since trees are connected graphs with no cycles, and this fact proves cyclicity, then it would prove that the graph is not actually a tree.
						if str(child) in y:
							# this marks the graph which isn't a tree in order to differentiate it from the rest later.
							y['invalid'] = 'invalid menu'

# this now deals with creating the final output based on if the dictionary has the keyword invalid in it, or not.
for values in general_list:
	if 'invalid' in values:
		for root in roots:
			if root in values:
				# this part of the logic loops through the dictionary and appends the id of the root to the root_id key, and appends the children (and descendents) to the child_id key
				temp_list = []
				for key in values:
					for num in values[key]:
						# doesn't append the value of the invalid marker
						if key != 'invalid':
							temp_list.append(num)
				temp_dict = {'root_id': root, 'children': temp_list}
				final_dict['invalid_menus'].append(temp_dict)
				
	# same as above except for the valid menus as there are no markers to suggest the tree is invalid
	else:
		for root in roots:
			if root in values:
				temp_list = []
				for key in values:
					for num in values[key]:
						temp_list.append(num)
				temp_dict = {'root_id': root, 'children': temp_list}
				final_dict['valid_menus'].append(temp_dict)
			
# convert to JSON notation
print(json.dumps(final_dict))