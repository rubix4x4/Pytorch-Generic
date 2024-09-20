# Utilizing Stable Diffusion to Generate Images for my DnD campaign
# This method makes the request to the stability ai website and has them generate it, so it costs money.
import requests
url = 'https://api.stability.ai/v2beta/stable-image/generate/core'
APIKey = 'sk-ZZqybiW0XAtUwV721fRXaFfXBN9kwMV92brGhjmexNxRI9k7'
Prompt_Script = "A middle aged half-orc male monk. Character protrait created in a drawn style"

response = requests.post(
    f"https://api.stability.ai/v2beta/stable-image/generate/sd3",
    headers = {     
        "authorization" : f"Bearer sk-ZZqybiW0XAtUwV721fRXaFfXBN9kwMV92brGhjmexNxRI9k7",    # Tells them who is requesting
        "accept": "image/*"                                                                 # Tells them what kind of file returns I'm accepting
    },
    
    files = {"none":''},
    
    data = {
        "prompt" : Prompt_Script,
        "mode" : 'text-to-image',
        "output_format": "jpeg"
    }
)

if response.status_code == 200:
    with open("./lighthouse.jpeg", 'wb') as file:
        file.write(response.content)
else:
    raise Exception(str(response.json()))

