#!/usr/bin/env python

import urwid, sys, os

'''
Console application for trianing and testing pick and place framework
Author: Luke Fraser
'''
class ObjectWalker(urwid.ListWalker):
    def __init__(self, object_dict):
        self.object_reference = object_dict
        self.focus = (0, 1)
    def _get_at_pos(self, pos):
        return urwid.Text(self.object_reference[pos[1]]), pos
    def get_focus(self):
        return self._get_at_pos(self.focus)
    def set_focus(self, focus):
        self.focus = focus
        self._modified()
    def get_next(self, start_from):
        a, b = start_from
        focus = b, a+b
        return self._get_at_pos(focus)
    def get_prev(self, start_from):
        a, b = start_from
        focus = b-a, a
        return self._get_at_pos(focus)


class TrainingObject(urwid.WidgetWrap):
    def __init__(self, object_name):
        urwid.WidgetWrap.__init__(self, self.MainWindow)
    def _button(self, t, fn):
        w = urwid.Button(t)
        urwid.connect_signal(w, 'click', fn, t)
        w = urwid.AttrWrap(w, 'button normal', 'button select')
        return w

class TrainingWindow(urwid.WidgetWrap):
    def __init__(self):
        urwid.WidgetWrap.__init__(self, self.MainWindow())
    def MainWindow(self):
        self.main_list = urwid.SimpleFocusListWalker([])
        main = urwid.ListBox([]self.main_list)
        return main
    def add(self, object_name):
        ok = self._button('ok', )
        columns = urwid.Columns()
        self.main_list.append()

class TrainingView(urwid.WidgetWrap):
    '''
    A Clss for displaying the interface.
    '''
    palette = [
        ('body',            'black',        'light gray',   'standout'),
        ('header',          'white',        'dark red',    'bold'),
        ('sceen edge',      'light blue',   'dark cyan'),
        ('main shadow',     'dark gray',    'black'),
        ('line',            'black',        'light gray',   'standout'),
        ('bg background',   'light gray',   'black'),
        ('bg 1',            'black',        'dark blue',    'standout'),
        ('bg 1 smooth',     'dark blue',    'black'),
        ('bg 2',            'black',        'dark cyan',    'standout'),
        ('bg 2 smooth',     'dark cyan',    'black'),
        ('button normal',   'light gray',   'dark blue',    'standout'),
        ('button select',   'white',        'dark green'),
        ('line',            'black',        'light gray',   'standout'),
        ('pg normal',       'white',        'black',        'standout'),
        ('pg complete',     'white',        'dark magenta'),
        ('pg smooth',       'dark magenta', 'black')
    ]
    def __init__(self):
        self.objects_ = [
            "neutral",
            "placemat",
            "cup",
            "plate",
            "fork",
            "spoon",
            "knife",
            "bowl",
            "soda",
            "wineglass"
        ]
        urwid.WidgetWrap.__init__(self, self.MainWindow())
    def OnTrain(self, button, name):
        '''
        This window is reponsible for training against all objects displayed
        within the list-box.
        '''
        for object in self.objects_list_walker:
            self.training_list.add(object.name)
    def OnLoad(self, button, filename):
        pass
    def OnSave(self, button, filename):
        pass
    def OnTest(self, button, object):
        pass
    def OnQuit(self, button, name):
        raise urwid.ExitMainLoop()

    def _ShadowWindow(self, view):
        bg     = urwid.AttrWrap(urwid.SolidFill(u"\u2592"), 'screen edge')
        shadow = urwid.AttrWrap(urwid.SolidFill(u" "), 'main shadow')

        bg = urwid.Overlay(shadow, bg,
            ('fixed left', 3), ('fixed right',  1),
            ('fixed top',  2), ('fixed bottom', 1))
        view = urwid.Overlay(view, bg,
            ('fixed left', 2), ('fixed right',  3),
            ('fixed top',  1), ('fixed bottom', 2))
        return view
    def _GenerateObjectList(self):
        text_list = [urwid.AttrMap(urwid.Text(x), None, focus_map='button select') for x in self.objects_]
        self.objects_list_walker = urwid.SimpleFocusListWalker(text_list)
        return urwid.ListBox(self.objects_list_walker)

    def _button(self, t, fn):
        w = urwid.Button(t)
        urwid.connect_signal(w, 'click', fn, t)
        w = urwid.AttrWrap(w, 'button normal', 'button select')
        return w
    def _GenerateOptions(self):
        self.button_list = []
        self.button_list.append(self._button(u"Train", self.OnTrain))
        self.button_list.append(self._button(u"Test", self.OnTest))
        self.button_list.append(self._button(u"Quit", self.OnQuit))
        return urwid.Filler(urwid.Columns(self.button_list))

    def MainWindow(self):
        # Generate List of objects
        self.objects_view = self._GenerateObjectList()

        # Generate Options list
        self.options_view = self._GenerateOptions()

        # Create Training window list
        self.training_list = TrainingWindow()
        # Create Main Window
        pile = urwid.Pile([('weight', 3, self.objects_view), self.options_view], focus_item=1)
        column = urwid.Columns([('weight', 3, pile), self.training_list], focus_column=0)
        view = urwid.Padding(column, left=2, right=2)
        return self._ShadowWindow(view)
    def OnTraining(self):
        '''
        This window is reponsible for training against all objects displayed
        within the list-box.
        '''
        #Create Training pop window for each object
        pass

    def main(self):
        self.loop = urwid.MainLoop(self, self.palette)
        self.loop.run()

def main():
    TrainingView().main()

if __name__ == '__main__':
    main()
