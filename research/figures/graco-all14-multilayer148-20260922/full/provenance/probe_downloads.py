import urllib.request,base64
share='AmSRwVTuK0YFghGt-5acB5oRN7qe'
b=base64.urlsafe_b64decode(share+'==');print(b.hex())
auth='!'+base64.urlsafe_b64encode(b[11:]).decode().rstrip('=')
urls=[f'https://onedrive.live.com/download?resid=05462BEE54C19164!273&authkey={auth}',
 'https://onedrive.live.com/download?resid=05462BEE54C19164!273&authkey=!AK37lpwHmhE3up4',
 'https://1drv.ms/u/s!AmSRwVTuK0YFghGt-5acB5oRN7qe?download=1',
 'https://api.onedrive.com/v1.0/shares/s!AmSRwVTuK0YFghGt-5acB5oRN7qe/root/content']
for u in urls:
 try:
  req=urllib.request.Request(u,headers={'User-Agent':'Mozilla/5.0','Range':'bytes=0-63'})
  with urllib.request.urlopen(req,timeout=20) as r: print(u,r.status,r.headers.get('Content-Type'),r.headers.get('Content-Length'),r.read(64))
 except Exception as e:print(u,repr(e))
