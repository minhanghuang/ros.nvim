local M = {}

local function generate_compile_commands()
  -- 当前工作区绝对路径
  local work_dir = vim.fn.getcwd()
  -- 当前文件绝对路径
  local curr_dir = vim.fn.fnamemodify(debug.getinfo(1, "S").source:sub(2), ":p:h")

  vim.fn.system('python3 ' .. curr_dir .. "/clangd.py " .. "-t " .. "compile_commands " .. "-ws " .. work_dir)
end

function M.setup(config)
  local conf = vim.deepcopy(config) or {}
  local enabled = conf.enabled or false
  local auto_generate_compile_commands = conf.auto_generate_compile_commands or false

  if not enabled then
    return
  end

  if auto_generate_compile_commands then
    generate_compile_commands()
  end
end

return M
