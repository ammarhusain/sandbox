import requests

attachments = []

# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_2_1.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_3_1.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_6_2.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_7_1.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_7_2.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_8_1.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_8_2.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_9_1.json'
# print attachment
# attachments.append(attachment)
# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_tile_v0/tile_2_10_1.json'
# print attachment
# attachments.append(attachment)

# attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_05_22/full_cloud.json'
# print attachment
# attachments.append(attachment)

#for frame in range(1734):
for frame in range(50):
  #attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_v0/cloud_000%s.json' % str(frame).zfill(2)
  attachment = 'https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_05_22_1/cloud_00000%s.json' % str(frame).zfill(3)

  print attachment
  attachments.append(attachment)

payload = {
  'callback_url': 'http://www.example.com/callback',
    'instruction': 'DO NOT LABEL; Please label 3D splines with drivable area',
  'attachment_type': 'json',
  'attachments': attachments,
  'labels': ['drivable']
}

headers = {"Content-Type": "application/json"}

task_request = requests.post("https://api.scaleapi.com/v1/task/lidarannotation",
  json=payload,
  headers=headers,
  auth=('live_818465b2c63d4133b57dc01eb417a5b6', ''))
  #auth=('test_376500fbe53d4902a105c1bee76920b0', ''))

print task_request.json()
