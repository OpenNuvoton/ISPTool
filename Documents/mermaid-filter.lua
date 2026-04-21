-- Pandoc Lua filter: render mermaid code blocks to PNG via mmdc
local count = 0

function CodeBlock(block)
  if block.classes[1] == 'mermaid' then
    count = count + 1
    local infile = os.tmpname()
    local outfile = '/tmp/mermaid_' .. count .. '.png'

    local f = io.open(infile, 'w')
    f:write(block.text)
    f:close()

    os.execute(string.format(
      'mmdc -i %s -o %s -b white --scale 2 -p /tmp/puppeteer-config.json 2>/dev/null',
      infile, outfile
    ))
    os.remove(infile)

    local img = io.open(outfile, 'rb')
    if img then
      img:close()
      local title = block.text:match('title:%s*"([^"]*)"') or ''
      return pandoc.Para({pandoc.Image({pandoc.Str(title)}, outfile)})
    end
    -- If mmdc failed, keep the original code block
  end
end
