function hideAxesToolbar(ax)
%HIDEAXESTOOLBAR Hide the interactive axes toolbar when available.

try
    if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
        ax.Toolbar.Visible = 'off';
    end
catch
end
end
