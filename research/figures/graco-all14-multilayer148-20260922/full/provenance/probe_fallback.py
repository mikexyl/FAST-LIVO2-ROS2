import urllib.request,base64,json
u='https://1drv.ms/f/c/05462bee54c19164/EmSRwVTuK0YggAV9AAAAAAABtpif_ADvF25CSv5v5RuvVg?e=9S9ixT'
s='u!'+base64.urlsafe_b64encode(u.encode()).decode().rstrip('=')
for endpoint in ['root/children','driveItem']:
 try:
  d=json.load(urllib.request.urlopen('https://api.onedrive.com/v1.0/shares/'+s+'/'+endpoint,timeout=20))
  print(endpoint,[(x.get('name'),x.get('id')) for x in d.get('value',[])]);open('/tmp/graco-fallback-'+endpoint.replace('/','-')+'.json','w').write(json.dumps(d))
 except Exception as e:print(endpoint,repr(e))
